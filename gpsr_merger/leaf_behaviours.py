"""py_trees leaf behaviours that wrap GPSR subtask-manager methods.

Each leaf calls the method that matches its action's ``.action`` field
(e.g. ``"go_to"`` → ``go_to(command)``) on the first handler in
``subtask_handlers`` that defines it — the same dispatch model used by
``gpsr_task_manager.search_command``.

The calls are synchronous. Subtask managers enforce the deadlines supplied
by the executor while the underlying subtask call is active.
"""

from typing import Any, Callable, List, Optional, Sequence, Tuple

import py_trees

from gpsr_merger.merger import PlanAction
from gpsr_merger.status import Status


def _resolve_method(action_name: str, handlers: Sequence[Any]) -> Optional[Callable]:
    for h in handlers:
        method = getattr(h, action_name, None)
        if callable(method):
            return method
    return None


def _handler_names(handlers: Sequence[Any]) -> str:
    return ", ".join(type(h).__name__ for h in handlers) or "<none>"


def _unpack_outcome(outcome: Any) -> Tuple[Optional[Status], Any]:
    """Return ``(status, result)`` from a handler return value.

    Subtask methods canonically return ``(Status, result)``; we also
    tolerate methods that return a bare ``Status``. Anything else (wrong
    type, empty tuple, non-Status first element) yields ``(None, raw)``
    so the caller can warn instead of silently mapping to FAILURE.
    """
    if isinstance(outcome, tuple):
        status = outcome[0] if outcome else None
        result = outcome[1] if len(outcome) > 1 else None
    else:
        status, result = outcome, None
    if not isinstance(status, Status):
        return None, outcome
    return status, result


def _dispatch(
    plan_action: PlanAction,
    method: Optional[Callable],
    on_complete: Optional[Callable[[PlanAction, Any, Any], None]],
    logger: Any,
    on_start: Optional[Callable[[PlanAction], None]] = None,
    is_late: Optional[Callable[[], bool]] = None,
    verify_success: Optional[Callable[[], bool]] = None,
) -> Status:
    """Call ``method(plan_action.action)``, fire ``on_complete``, return Status.

    Converts missing handlers, exceptions and invalid outcomes into failures.
    """
    if on_start is not None:
        try:
            on_start(plan_action)
        except Exception:  # noqa: BLE001
            logger.error("on_start hook raised")

    action = plan_action.action
    kind = getattr(action, "action", "")
    status, result = Status.EXECUTION_ERROR, None
    if method is None:
        logger.error(f"No handler for action '{kind}'")
    else:
        try:
            outcome = method(action)
        except Exception:  # noqa: BLE001 — we want the BT to handle it
            logger.error(f"{kind} raised")
        else:
            parsed_status, result = _unpack_outcome(outcome)
            if parsed_status is None:
                logger.warning(f"{kind} returned non-Status outcome; treating as failure")
            else:
                status = parsed_status
    if is_late is not None and is_late():
        status, result = Status.TIMEOUT, None
    elif status == Status.EXECUTION_SUCCESS and verify_success is not None and not verify_success():
        status, result = Status.EXECUTION_ERROR, None
    if on_complete is not None:
        try:
            on_complete(plan_action, status, result)
        except Exception:  # noqa: BLE001
            logger.error("on_complete hook raised")
    return status


class ActionLeaf(py_trees.behaviour.Behaviour):
    """Leaf that runs one ``PlanAction`` to completion."""

    def __init__(
        self,
        plan_action: PlanAction,
        subtask_handlers: Sequence[Any],
        on_complete: Optional[Callable[[PlanAction, Any, Any], None]] = None,
        name: Optional[str] = None,
        on_start: Optional[Callable[[PlanAction], None]] = None,
    ):
        kind = getattr(plan_action.action, "action", "?")
        super().__init__(name=name or f"{kind}#{plan_action.source_cmd}.{plan_action.source_idx}")
        self._plan_action = plan_action
        self._handlers = subtask_handlers
        self._method = _resolve_method(kind, subtask_handlers)
        self._on_complete = on_complete
        self._on_start = on_start

    def update(self) -> py_trees.common.Status:
        if self._method is None:
            kind = getattr(self._plan_action.action, "action", "")
            self.logger.error(
                f"No handler for action '{kind}' in [{_handler_names(self._handlers)}]"
            )
            return py_trees.common.Status.FAILURE
        status = _dispatch(
            self._plan_action,
            self._method,
            self._on_complete,
            self.logger,
            on_start=self._on_start,
        )
        if status == Status.EXECUTION_SUCCESS:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


def fallback_order(
    fallback: Sequence[Sequence[PlanAction]],
    is_completed: Optional[Callable[[PlanAction], bool]],
    *,
    holder: Optional[int] = None,
) -> List[int]:
    """Issue order, except the command holding an object runs right before the
    first other command that still has to pick (one gripper)."""
    order = list(range(len(fallback)))
    if is_completed is None:
        return order
    if holder is None:
        for c, actions in enumerate(fallback):
            held = False
            for pa in actions:
                if pa.requires_gripper and is_completed(pa):
                    held = True
                if pa.releases_gripper and is_completed(pa):
                    held = False
            if held:
                holder = c
    if holder is None:
        return order
    pickers = [c for c in order if c != holder
               and any(pa.requires_gripper and not is_completed(pa) for pa in fallback[c])]
    if pickers and pickers[0] < holder:
        order.remove(holder)
        order.insert(order.index(pickers[0]), holder)
    return order


def _resume_skip_mask(
    actions: Sequence[PlanAction],
    is_completed: Optional[Callable[[PlanAction], bool]],
) -> List[bool]:
    """Which actions of one command to skip when resuming.

    A non-``go_to`` the interleaved branch already finished is skipped, and a
    ``go_to`` is skipped when every action up to the next ``go_to`` is.
    """
    def is_go_to(pa: PlanAction) -> bool:
        return getattr(pa.action, "action", "") == "go_to"

    skip = [not is_go_to(pa) and is_completed is not None and is_completed(pa) for pa in actions]
    for i, pa in enumerate(actions):
        if not is_go_to(pa):
            continue
        pending = False
        for later in range(i + 1, len(actions)):
            if is_go_to(actions[later]):
                break
            if not skip[later]:
                pending = True
                break
        skip[i] = not pending
    return skip


class SequentialFallbackLeaf(py_trees.behaviour.Behaviour):
    """Runs every command's fallback sequence, one command after another, in
    ``fallback_order``. Failures are logged, not propagated, so every command
    gets its chance; always returns SUCCESS."""

    def __init__(
        self,
        fallback: Sequence[Sequence[PlanAction]],
        subtask_handlers: Sequence[Any],
        on_complete: Optional[Callable[[PlanAction, Any, Any], None]] = None,
        name: str = "fallback",
        is_completed: Optional[Callable[[PlanAction], bool]] = None,
        on_start: Optional[Callable[[PlanAction], None]] = None,
    ):
        super().__init__(name=name)
        self._fallback = fallback
        self._handlers = subtask_handlers
        self._on_complete = on_complete
        self._is_completed = is_completed
        self._on_start = on_start
        # Resolved now so a missing handler shows up at tree construction.
        self._methods = [
            [_resolve_method(getattr(pa.action, "action", ""), subtask_handlers) for pa in actions]
            for actions in fallback
        ]

    def update(self) -> py_trees.common.Status:
        for c in fallback_order(self._fallback, self._is_completed):
            actions = self._fallback[c]
            if not actions:
                continue
            self.logger.info(f"fallback running cmd_idx={c}, {len(actions)} actions")
            skip = _resume_skip_mask(actions, self._is_completed)
            for pa, method, skipped in zip(actions, self._methods[c], skip):
                kind = getattr(pa.action, "action", "")
                if skipped:
                    self.logger.info(f"resume: skipping {kind} cmd={pa.source_cmd} idx={pa.source_idx}")
                    continue
                if method is None:
                    self.logger.error(
                        f"No handler for action '{kind}' in [{_handler_names(self._handlers)}]"
                    )
                    continue
                _dispatch(pa, method, self._on_complete, self.logger, on_start=self._on_start)
        return py_trees.common.Status.SUCCESS


class OneShotCallbackLeaf(py_trees.behaviour.Behaviour):
    """Leaf that fires ``callback`` exactly once on its first update.

    Used at the head of the sequential-fallback branch so the operator gets
    a visible signal (on-screen text, log line, etc.) the moment the tree
    advances past the interleaved branch. Always returns SUCCESS so the
    enclosing Sequence advances unconditionally.
    """

    def __init__(self, callback: Callable[[], None], name: str = "one_shot"):
        super().__init__(name=name)
        self._callback = callback
        self._fired = False

    def update(self) -> py_trees.common.Status:
        if not self._fired:
            self._fired = True
            try:
                self._callback()
            except Exception:  # noqa: BLE001 — never let a debug hook tank the BT
                self.logger.error(f"{self.name} callback raised")
        return py_trees.common.Status.SUCCESS
