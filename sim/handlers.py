"""``SimSubtaskHandler`` — the simulated GPSR subtask manager.

The executor calls skills by action kind and expects ``(Status, result)``.
The adapter also exposes the verified gripper state, return and reporting
operations, and cost estimates for deferral. Robot steps enforce
``robot.deadline`` inside the blocking skill and return calls.

Every movement is planned by ``sim.routing`` (the stand-in for Nav2): paths go
around furniture and around people other than the one being escorted.
"""

import math
from typing import Any, Dict, List, Optional, Tuple

from gpsr_merger.recovery import GripperState
from gpsr_merger.status import Status
from gpsr_merger.timeouts import timeout_for
from sim.robot import ANG_SPEED, DT, EPS_M, LIN_SPEED, SimRobot, SimTimeout, wrap
from sim.routing import Router, VisGraphRouter, path_length
from sim.world import SimObject, SimPerson, World

SENSE_RADIUS = 3.2      # m — perception range for find/count/visual-info
REACH_RADIUS = 2.4      # m — the robot must be at the object's location to pick
HANDOVER_RADIUS = 2.6   # m — a person must be close to receive an object
PICK_STANDOFF = 0.55    # m — how close the base gets to an object it picks
FAILURE_REPORT_S = 2.6

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
                 failures: Optional[Dict[str, List[Status]]] = None,
                 router: Optional[Router] = None, skill_model=None):
        self.world = world
        self.skill_model = skill_model
        self.minimum_finish = None
        self.robot = robot
        self.router = router if router is not None else VisGraphRouter(world)
        self.recorder = recorder
        self.failures = dict(failures or {})
        self.site = world.nearest_site(robot.xy)
        self.current = None              # PlanAction, set by on_action_start
        self.locked: Dict[int, SimPerson] = {}   # per source command
        self.results: Dict[int, Dict[int, str]] = {}

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

    def gripper_state(self) -> GripperState:
        return GripperState.EMPTY if self.robot.held is None else GripperState.OCCUPIED

    def set_action_deadline(self, deadline: Optional[float]) -> None:
        self.robot.deadline = deadline

    def _travel_time(self, start, goal, yaw, goal_yaw):
        path = self._path(start, goal)
        if path is None or World.distance(path[-1] if path else start, goal) > EPS_M:
            return None
        # Round each motion to simulation ticks. A skill may leave an unknown yaw.
        elapsed = 0.0
        for point in path:
            distance = World.distance(start, point)
            if distance > EPS_M:
                heading = math.atan2(point[1] - start[1], point[0] - start[0])
                angle = math.pi if yaw is None else abs(wrap(heading - yaw))
                elapsed += (math.ceil(angle / (ANG_SPEED * DT))
                            + math.ceil(distance / (LIN_SPEED * DT))) * DT
                yaw = heading
            start = point
        angle = math.pi if yaw is None else abs(wrap(goal_yaw - yaw))
        return elapsed + (math.ceil(angle / (ANG_SPEED * DT)) + 1) * DT

    def estimate_deferred(self, steps, home) -> Optional[float]:
        """Price the revisit and reserve a route home even if a skill fails."""
        xy, yaw = self.robot.xy, self.robot.yaw
        elapsed, estimate = 0.0, 0.0
        for pa in [*steps, None]:
            home_time = self._travel_time(xy, home[:2], yaw, home[2])
            if home_time is None:
                return None
            estimate = max(estimate, elapsed + home_time)
            if pa is None:
                break
            kind = pa.action.action
            if kind == "go_to":
                pose = self.world.pose(pa.action.location_to_go)
                travel = None if pose is None else self._travel_time(xy, pose[:2], yaw, pose[2])
                if travel is None or travel > timeout_for(kind):
                    return None
                elapsed += max(travel, 0.6)  # includes the cost of an injected failure
                xy, yaw = pose[:2], pose[2]
            else:
                elapsed += {
                    "count": 2 * DURATION["count"] + DT,
                    "get_visual_info": 2 * DURATION["get_visual_info"] + DT,
                    "find_person": DURATION["find_person"] + 1.4 + math.pi / ANG_SPEED + DT,
                    "find_person_by_name": DURATION["find_person_by_name"] + 1.4 + math.pi / ANG_SPEED + DT,
                    "get_person_info": DURATION["get_person_info"] + math.pi / ANG_SPEED + DT,
                    "say_with_context": DURATION["say_with_context"],
                }[kind]
                yaw = None
        return estimate

    def return_to_start(self, home, deadline: float) -> bool:
        self.robot.deadline = deadline
        try:
            path = self._path(self.robot.xy, home[:2])
            if path is None:
                return False
            self.robot.drive_path(path, yaw=home[2])
            return World.distance(self.robot.xy, home[:2]) <= EPS_M
        except SimTimeout:
            return False
        finally:
            self.robot.deadline = None

    def report_failure(self, plan_action, status: Status, deadline: float) -> bool:
        self.current = plan_action
        self.robot.deadline = deadline
        try:
            text = f"Command {self.cmd_idx + 1}: could not complete {plan_action.action.action}."
            self._say(text, anchor="robot", hold=FAILURE_REPORT_S)
            if self.recorder is not None:
                self.recorder.failure_report(plan_action, status, text, self.robot)
            return True
        except SimTimeout:
            return False
        finally:
            self.robot.deadline = None

    # ---- internals -------------------------------------------------------

    @property
    def cmd_idx(self) -> int:
        return getattr(self.current, "source_cmd", -1)

    def _injected(self, kind: str) -> Optional[Status]:
        """Pop a scripted failure for this action, if one is queued.

        Keys are ``"<cmd>.<idx>"`` (precise) or a bare action kind.
        """
        for key in (f"{getattr(self.current, 'source_cmd', -1)}."
                    f"{getattr(self.current, 'source_idx', -1)}", kind):
            queue = self.failures.get(key)
            if queue:
                return queue.pop(0)
        return None

    def _begin(self, kind: str):
        skill_deadline = self.robot.t + timeout_for(kind)
        if self.robot.deadline is not None:
            skill_deadline = min(skill_deadline, self.robot.deadline)
        self.robot.deadline = skill_deadline

    def _end(self, status: Status, result=None) -> Tuple[Status, Any]:
        if status == Status.EXECUTION_SUCCESS and self.minimum_finish is not None:
            wait = self.minimum_finish - self.robot.t
            if wait > DT / 2:
                self.robot.idle(wait)
        self.robot.deadline = None
        kind = getattr(getattr(self.current, "action", None), "action", "")
        if (status == Status.EXECUTION_SUCCESS and result is not None
                and getattr(self.current, "purpose", None) != "reacquire"
                and kind not in ("go_to", "say_with_context")):
            self.results.setdefault(self.cmd_idx, {})[self.current.source_idx] = str(result)
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

    def _path(self, start, goal, exclude: Optional[SimPerson] = None):
        """Collision-free waypoints ``start`` -> ``goal`` given where everyone
        (except ``exclude``) is standing right now; ``None`` if unreachable."""
        self.router.set_people([self.robot.person_xy(p) for p in self.world.people
                                if p is not exclude])
        return self.router.plan_path(start, goal)

    def _say(self, text: str, anchor: Optional[str] = None, hold: float = 2.2) -> None:
        if getattr(self.current, "purpose", None) not in ("reacquire", "deferred"):
            self.robot.speech = text
            self.robot.speech_anchor = anchor
        self.robot.idle(hold)

    # ---- guard shared by all 13 leaves -----------------------------------

    def _run(self, kind: str, body):
        self.minimum_finish = None
        injected = self._injected(kind)
        self._begin(kind)
        try:
            if self.skill_model is not None:
                ok, duration = self.skill_model.sample(self.cmd_idx, self.current.source_idx, kind)
                if not ok:
                    self.robot.idle(0.6 if duration is None else duration)
                    return self._end(Status.EXECUTION_ERROR, None)
                if duration is not None:
                    self.minimum_finish = self.robot.t + duration
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
            path = self._path(self.robot.xy, pose[:2])
            if path is None:
                self.robot.idle(0.5)
                return self._end(Status.EXECUTION_ERROR, None)
            self.robot.drive_path(path, yaw=pose[2])
            self.site = name
            return self._end(Status.EXECUTION_SUCCESS, name)
        return self._run("go_to", body)

    def pick_object(self, command) -> Tuple[Status, Any]:
        def body():
            name = _f(command, "object_to_pick", "")
            if self.robot.held is not None:
                # Single-item invariant, enforced by the executor.
                return self._end(Status.EXECUTION_ERROR, None)
            obj = self.world.find_object(name)
            if obj is None or not self.world.observes_site(self.site, obj.site):
                self.robot.idle(0.5)
                return self._end(Status.TARGET_NOT_FOUND, None)
            oxyz = self.robot.object_xyz(obj)
            if not self._near(oxyz[:2], REACH_RADIUS):
                self.robot.idle(0.5)
                return self._end(Status.TARGET_NOT_FOUND, None)
            (ox, oy), (rx, ry) = oxyz[:2], self.robot.xy
            d = math.hypot(rx - ox, ry - oy)
            if d > PICK_STANDOFF:
                # Stand off on the robot's side of the object; the router moves
                # that point out of the furniture the object sits on.
                k = PICK_STANDOFF / d
                path = self._path((rx, ry), (ox + (rx - ox) * k, oy + (ry - oy) * k))
                if path is None:
                    return self._end(Status.EXECUTION_ERROR, None)
                self.robot.drive_path(path)
            self._face(oxyz[:2])
            self.robot.grasp(obj)
            obj.site = None
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
            obj.site = self.site
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
            obj.site = self.site
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
            # The person walks their own route; the robot first drives to where
            # they stood, then retraces that route behind them.
            start = self.robot.person_xy(person)
            reach = self._path(self.robot.xy, start, exclude=person)
            walk = self._path(start, pose[:2], exclude=person)
            if reach is None or walk is None:
                return self._end(Status.EXECUTION_ERROR, None)
            lead_in = [self.robot.xy] + reach[:-1] + [start]
            self.robot.follow_person(person, lead_in + walk,
                                     person_from=path_length(lead_in[0], lead_in[1:]))
            self.site = dest
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
            # The person first walks to where the robot is, then retraces the
            # robot's route behind it.
            start = self.robot.person_xy(person)
            drive = self._path(self.robot.xy, pose[:2], exclude=person)
            join = self._path(start, self.robot.xy, exclude=person)
            if drive is None or join is None:
                return self._end(Status.EXECUTION_ERROR, None)
            lead_in = [start] + join[:-1] + [self.robot.xy]
            self.robot.lead_person(person, lead_in + drive,
                                   robot_from=path_length(lead_in[0], lead_in[1:]))
            self.site = dest
            return self._end(Status.EXECUTION_SUCCESS, dest)
        return self._run("guide_person_to", body)

    def _visible_objects(self, target):
        return [obj for obj in self.world.objects_matching(target)
                if self.world.observes_site(self.site, obj.site)
                and self._near(self.robot.object_xyz(obj)[:2], SENSE_RADIUS)]

    def count(self, command) -> Tuple[Status, Any]:
        def body():
            target = _f(command, "target_to_count", "")
            self.robot.scan(duration=DURATION["count"])
            words = target.lower().split()
            people = {"people", "persons", "person"}
            if people.intersection(words):
                attribute = " ".join(word for word in words if word not in people)
                hits = [p for p in self._people_in_range()
                        if self.world.observes_site(self.site, self.world.nearest_site(self.robot.person_xy(p)))
                        and (not attribute or any(attribute in attr.lower() for attr in p.attributes))]
            else:
                hits = self._visible_objects(target)
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
            hits = self._visible_objects(category)
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
            history = self.results.get(self.cmd_idx, {})
            prior = [result for idx, result in sorted(history.items())
                     if self.current is None or idx < self.current.source_idx]
            if not prior:
                prior = _f(command, "previous_command_info", [])
            ctx = "; ".join(str(p) for p in prior if p)
            text = f"About {instruction}: {ctx}" if ctx else f"About {instruction}."
            self._say(text, anchor="robot", hold=DURATION["say_with_context"])
            return self._end(Status.EXECUTION_SUCCESS, text)
        return self._run("say_with_context", body)
