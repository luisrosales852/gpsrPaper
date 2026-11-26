"""Paired, per-attempt outcomes and empirical skill durations for GPSR-Sim."""

import math
import random


class SkillModel:
    def __init__(self, seed, failure_rate=0.0, skills=None):
        if not 0 <= failure_rate <= 1:
            raise ValueError("failure_rate must be between zero and one")
        self.seed = seed
        self.failure_rate = failure_rate
        self.skills = skills or {}
        self.attempts = {}

    def sample(self, cmd, idx, kind):
        key = (cmd, idx, kind)
        attempt = self.attempts.get(key, 0)
        self.attempts[key] = attempt + 1
        rng = random.Random(f"{self.seed}/{cmd}/{idx}/{kind}/{attempt}")
        skill = self.skills.get(kind, {})
        ok = rng.random() >= skill.get("failure_rate", self.failure_rate)
        durations = skill.get("success_s" if ok else "failure_s", ())
        return ok, rng.choice(durations) if durations else None


def fit_skill_model(records):
    """Fit from one row per observed attempt, including timeouts and failures."""
    skills = {}
    for row in records:
        duration = float(row["duration_s"])
        if not math.isfinite(duration) or duration < 0 or type(row["success"]) is not bool:
            raise ValueError("each observation needs a boolean success and finite nonnegative duration_s")
        skill = skills.setdefault(row["skill"], {"success_s": [], "failure_s": []})
        skill["success_s" if row["success"] else "failure_s"].append(duration)
    for skill in skills.values():
        failures = len(skill["failure_s"])
        skill["failure_rate"] = failures / (failures + len(skill["success_s"]))
    return skills
