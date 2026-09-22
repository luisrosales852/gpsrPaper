"""``SimSubtaskHandler`` — the simulated GPSR subtask manager.

This is the whole integration surface. ``leaf_behaviours._resolve_method`` looks
up a method whose *name* equals the action kind and calls it with the BAML
command object, expecting ``(Status, result)`` back — so implementing the 13
kinds below is sufficient to drive the real behaviour tree. No subclassing, no
registration, no patching of ``gpsr_merger``.

Two properties make the run evidence rather than decoration:

* **The gripper is enforced here, in the executor, not only in the tests.**
  ``pick_object`` hard-fails when something is already held, so a schedule that
  violated the single-item invariant would visibly break the run. That is an
  execution-level witness for ``tests._merger_helpers.gripper_invariant_holds``.
* **Timeouts are enforced in sim-time by the handler**, via ``robot.deadline``
  and ``timeouts.timeout_for`` — mirroring the real subtask managers rather than
  relying on py_trees' between-tick ``Timeout`` decorator.
"""

import math
from typing import Any, Dict, List, Optional, Tuple

from gpsr_merger.status import Status
from gpsr_merger.timeouts import timeout_for
from sim.robot import SimRobot, SimTimeout
from sim.world import SimObject, SimPerson, World

SENSE_RADIUS = 3.2      # m — perception range for find/count/visual-info
REACH_RADIUS = 2.4      # m — the robot must be at the object's location to pick
HANDOVER_RADIUS = 2.6   # m — a person must be close to receive an object

# Sim-seconds each non-navigation leaf takes. Chosen under the corresponding
# budgets in gpsr_merger/timeouts.py so a nominal run never trips a timeout.
DURATION = {
    "pick_object": 2.6,
    "place_object": 2.0,
    "give_object": 2.2,
    "find_person": 2.6,
    "find_person_by_name": 2.6,
    "get_person_info": 2.4,
    "count": 2.8,
    "get_visual_info": 2.6,
    "answer_question": 3.0,
    "say_with_context": 2.6,
}


def _f(command: Any, field: str, default=None):
    v = getattr(command, field, default)
    return default if v is None else v


class SimSubtaskHandler:
    def __init__(self, world: World, robot: SimRobot, recorder=None,
                 failures: Optional[Dict[str, List[Status]]] = None):
        self.world = world
        self.robot = robot
        self.recorder = recorder
        self.failures = dict(failures or {})
        self.current = None              # PlanAction, set by on_action_start
        self.locked: Dict[int, SimPerson] = {}   # per source command
        self.last_results: List[str] = []

    # ---- wiring from bt_builder hooks ------------------------------------

    def on_action_start(self, plan_action) -> None:
        """Wired to ``build_tree(on_action_start=...)``.

        The leaf only hands handlers the bare action object, so this is where the
        provenance (``source_cmd``/``source_idx``) arrives — exactly how
        ``GPSRTM._on_action_start`` learns it on the real robot.
        """
        self.current = plan_action
        self.robot.clear_annotations()
        if self.recorder is not None:
            self.recorder.action_start(plan_action, self.robot)

    def on_action_complete(self, plan_action, status, result) -> None:
        if self.recorder is not None:
            self.recorder.action_end(plan_action, status, result, self.robot)

    # ---- internals -------------------------------------------------------

    @property
    def cmd_idx(self) -> int:
        return getattr(self.current, "source_cmd", -1)

    def _injected(self, kind: str) -> Optional[Status]:
        """Pop a scripted failure for this action, if one is queued.

        Keys are ``"<cmd>.<idx>"`` (precise) or a bare action kind (every
        occurrence). Used to exercise Retry -> Selector -> sequential_fallback.
        """
        for key in (f"{getattr(self.current, 'source_cmd', -1)}."
                    f"{getattr(self.current, 'source_idx', -1)}", kind):
            queue = self.failures.get(key)
            if queue:
                return queue.pop(0)
        return None

    def _begin(self, kind: str):
        self.robot.deadline = self.robot.t + timeout_for(kind)

    def _end(self, status: Status, result=None) -> Tuple[Status, Any]:
        self.robot.deadline = None
        if status == Status.EXECUTION_SUCCESS and result is not None:
            self.last_results.append(str(result))
        return (status, result)

    def _near(self, xy, radius: float) -> bool:
        return World.distance(self.robot.xy, xy) <= radius

    def _people_in_range(self, radius: float = SENSE_RADIUS) -> List[SimPerson]:
        ps = [p for p in self.world.people if self._near(self.robot.person_xy(p), radius)]
        return sorted(ps, key=lambda p: World.distance(self.robot.xy, self.robot.person_xy(p)))

    def _target_person(self) -> Optional[SimPerson]:
        """The person this command is working with.

        Prefers the one locked by an earlier ``find_person*`` *in the same source
        command* — which is what makes the per-command ordering invariant
        observable: if the merger ever emitted a follow before its find, this
        would fall back and the event log would show it.
        """
        p = self.locked.get(self.cmd_idx)
        if p is not None:
            return p
        near = self._people_in_range(SENSE_RADIUS)
        return near[0] if near else None

    def _face(self, xy) -> None:
        self.robot.turn_to(math.atan2(xy[1] - self.robot.xy[1], xy[0] - self.robot.xy[0]))

    def _say(self, text: str, anchor: Optional[str] = None, hold: float = 2.2) -> None:
        self.robot.speech = text
        self.robot.speech_anchor = anchor
        self.robot.idle(hold)

    # ---- guard shared by all 13 leaves -----------------------------------

    def _run(self, kind: str, body):
        injected = self._injected(kind)
        self._begin(kind)
        try:
            if injected is not None:
                self.robot.idle(0.6)
                return self._end(injected, None)
            return body()
        except SimTimeout:
            return self._end(Status.TIMEOUT, None)

    # =====================================================================
    # The 13 leaf behaviours
    # =====================================================================

    def go_to(self, command) -> Tuple[Status, Any]:
        def body():
            name = _f(command, "location_to_go", "")
            pose = self.world.pose(name)
            if pose is None:
                self.robot.idle(0.5)
                return self._end(Status.TARGET_NOT_FOUND, None)
            self.robot.drive_to(pose[0], pose[1], yaw=pose[2])
            return self._end(Status.EXECUTION_SUCCESS, name)
        return self._run("go_to", body)

    def pick_object(self, command) -> Tuple[Status, Any]:
        def body():
            name = _f(command, "object_to_pick", "")
            if self.robot.held is not None:
                # Single-item invariant, enforced by the executor.
                return self._end(Status.EXECUTION_ERROR, None)
            obj = self.world.find_object(name)
            if obj is None:
                self.robot.idle(0.5)
                return self._end(Status.TARGET_NOT_FOUND, None)
            oxyz = self.robot.object_xyz(obj)
            if not self._near(oxyz[:2], REACH_RADIUS):
                self.robot.idle(0.5)
                return self._end(Status.TARGET_NOT_FOUND, None)
            self._face(oxyz[:2])
            self.robot.drive_to(oxyz[0], oxyz[1], stop_within=0.55)
            self._face(oxyz[:2])
            self.robot.grasp(obj)
            self.robot.idle(DURATION["pick_object"] * 0.4)
            return self._end(Status.EXECUTION_SUCCESS, obj.kind)
        return self._run("pick_object", body)

    def place_object(self, command) -> Tuple[Status, Any]:
        def body():
            if self.robot.held is None:
                return self._end(Status.EXECUTION_ERROR, None)
            obj = self.robot.held
            x, y = self.robot.xy
            yaw = self.robot.yaw
            drop = (x + 0.65 * math.cos(yaw), y + 0.65 * math.sin(yaw), 0.75)
            self.robot.release(drop)
            self.robot.idle(DURATION["place_object"] * 0.5)
            return self._end(Status.EXECUTION_SUCCESS, obj.kind)
        return self._run("place_object", body)

    def give_object(self, command) -> Tuple[Status, Any]:
        def body():
            if self.robot.held is None:
                return self._end(Status.EXECUTION_ERROR, None)
            obj = self.robot.held
            near = self._people_in_range(HANDOVER_RADIUS)
            person = self.locked.get(self.cmd_idx) or (near[0] if near else None)
            if person is None or not self._near(self.robot.person_xy(person), HANDOVER_RADIUS):
                return self._end(Status.TARGET_NOT_FOUND, None)
            px, py = self.robot.person_xy(person)
            self._face((px, py))
            self.robot.release((px, py, 1.05))
            self._say(f"Here is the {obj.kind}, {person.name}.", anchor="robot",
                      hold=DURATION["give_object"])
            return self._end(Status.EXECUTION_SUCCESS, f"{obj.kind} to {person.name}")
        return self._run("give_object", body)

    def find_person(self, command) -> Tuple[Status, Any]:
        def body():
            attr = _f(command, "attribute_value", "")
            self.robot.scan(duration=DURATION["find_person"])
            person = self.world.person_by_attribute(attr)
            if person is None or not self._near(self.robot.person_xy(person), SENSE_RADIUS):
                return self._end(Status.TARGET_NOT_FOUND, None)
            self.locked[self.cmd_idx] = person
            self.robot.highlight_person(person, True)
            self._face(self.robot.person_xy(person))
            self._say(f"Found a person {attr}.", anchor="robot", hold=1.4)
            return self._end(Status.EXECUTION_SUCCESS, person.name)
        return self._run("find_person", body)

    def find_person_by_name(self, command) -> Tuple[Status, Any]:
        def body():
            name = _f(command, "name", "")
            self.robot.scan(duration=DURATION["find_person_by_name"])
            person = self.world.person_by_name(name)
            if person is None or not self._near(self.robot.person_xy(person), SENSE_RADIUS):
                return self._end(Status.TARGET_NOT_FOUND, None)
            self.locked[self.cmd_idx] = person
            self.robot.highlight_person(person, True)
            self._face(self.robot.person_xy(person))
            self._say(f"Hello {person.name}.", anchor="robot", hold=1.4)
            return self._end(Status.EXECUTION_SUCCESS, person.name)
        return self._run("find_person_by_name", body)

    def get_person_info(self, command) -> Tuple[Status, Any]:
        def body():
            info_type = str(_f(command, "info_type", "name"))
            person = self._target_person()
            if person is None:
                return self._end(Status.TARGET_NOT_FOUND, None)
            self.robot.highlight_person(person, True)
            self._face(self.robot.person_xy(person))
            answer = person.info.get(info_type, person.info.get("name", person.name))
            self._say(f"{info_type}: {answer}", anchor=person.body,
                      hold=DURATION["get_person_info"])
            return self._end(Status.EXECUTION_SUCCESS, answer)
        return self._run("get_person_info", body)

    def follow_person_until(self, command) -> Tuple[Status, Any]:
        def body():
            dest = _f(command, "destination", "")
            pose = self.world.pose(dest)
            person = self._target_person()
            if pose is None or person is None:
                self.robot.idle(0.5)
                return self._end(Status.TARGET_NOT_FOUND, None)
            self.robot.highlight_person(person, True)
            self._say(f"Following you to the {dest}.", anchor="robot", hold=1.2)
            self.robot.walk_person(person, (pose[0], pose[1]), robot_follows=True)
            return self._end(Status.EXECUTION_SUCCESS, dest)
        return self._run("follow_person_until", body)

    def guide_person_to(self, command) -> Tuple[Status, Any]:
        def body():
            dest = _f(command, "destination_room", "")
            pose = self.world.pose(dest)
            person = self._target_person()
            if pose is None or person is None:
                self.robot.idle(0.5)
                return self._end(Status.TARGET_NOT_FOUND, None)
            self.robot.highlight_person(person, True)
            self._say(f"Please follow me to the {dest}.", anchor="robot", hold=1.2)
            self.robot.lead_person(person, (pose[0], pose[1]))
            return self._end(Status.EXECUTION_SUCCESS, dest)
        return self._run("guide_person_to", body)

    def count(self, command) -> Tuple[Status, Any]:
        def body():
            target = _f(command, "target_to_count", "")
            self.robot.scan(duration=DURATION["count"])
            hits = [o for o in self.world.objects_matching(target)
                    if self._near(self.robot.object_xyz(o)[:2], SENSE_RADIUS)]
            self.robot.badges = [
                {"body": o.body, "text": str(i + 1), "color": (250, 210, 60)}
                for i, o in enumerate(hits)
            ]
            self._say(f"I counted {len(hits)} {target}.", anchor="robot",
                      hold=DURATION["count"])
            return self._end(Status.EXECUTION_SUCCESS, f"{len(hits)} {target}")
        return self._run("count", body)

    def get_visual_info(self, command) -> Tuple[Status, Any]:
        def body():
            measure = str(_f(command, "measure", "biggest")).lower()
            category = _f(command, "object_category", "object")
            self.robot.scan(duration=DURATION["get_visual_info"])
            hits = [o for o in self.world.objects_matching(category)
                    if self._near(self.robot.object_xyz(o)[:2], SENSE_RADIUS)]
            if not hits:
                return self._end(Status.TARGET_NOT_FOUND, None)
            reverse = measure in ("biggest", "largest", "heaviest", "tallest")
            winner = sorted(hits, key=lambda o: o.size, reverse=reverse)[0]
            self.robot.badges = [{"body": winner.body, "text": measure, "color": (90, 200, 255)}]
            self._say(f"The {measure} {category} is the {winner.kind}.", anchor="robot",
                      hold=DURATION["get_visual_info"])
            return self._end(Status.EXECUTION_SUCCESS, f"{measure} {category}: {winner.kind}")
        return self._run("get_visual_info", body)

    def answer_question(self, command) -> Tuple[Status, Any]:
        def body():
            person = self._target_person()
            if person is not None:
                self.robot.highlight_person(person, True)
                self._face(self.robot.person_xy(person))
            answer = "It is a GPSR task: I merge several commands into one plan."
            self._say(answer, anchor="robot", hold=DURATION["answer_question"])
            return self._end(Status.EXECUTION_SUCCESS, answer)
        return self._run("answer_question", body)

    def say_with_context(self, command) -> Tuple[Status, Any]:
        def body():
            instruction = str(_f(command, "user_instruction", ""))
            prior = _f(command, "previous_command_info", []) or self.last_results[-2:]
            ctx = "; ".join(str(p) for p in prior if p)
            text = f"About {instruction}: {ctx}" if ctx else f"About {instruction}."
            self._say(text, anchor="robot", hold=DURATION["say_with_context"])
            return self._end(Status.EXECUTION_SUCCESS, text)
        return self._run("say_with_context", body)
