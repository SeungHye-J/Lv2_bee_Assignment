"""가상 미션 · 기준선 확인 — python3 -m pytest test_mission_sim.py (또는 python3 test_mission_sim.py)"""
import numpy as np
from gymnasium.utils.env_checker import check_env

from baseline import baseline_action
from mission_sim import PIPS, RETURN, MissionSim, random_problem


def play(env, choose, seed):
    env.reset(seed=seed)
    for _ in range(1000):
        _, _, term, trunc, info = env.step(choose(env))
        if term or trunc:
            return info
    raise AssertionError("끝나지 않음")


def test_gym_api():
    check_env(MissionSim(seed=0), skip_render_check=True)


def test_problem_is_valid():
    rng = np.random.default_rng(1)
    for _ in range(200):
        t = random_problem(rng)
        assert len(t) == 12 and len(set(t)) == 12
        for loc in range(3):
            cells = sorted(c for l, c in t if l == loc)
            assert cells == sorted(PIPS[len(cells)])


def test_baseline_perfect_when_skills_never_fail():
    sure = {"lying_at_start": 0.0, "pick": 1.0, "stand": 1.0, "load": 1.0, "unload": 1.0, "place_ok": 1.0,
            "place_ok_center_after_neighbours": 1.0, "place_off": 0.0, "place_lying": 0.0}
    for seed in range(20):
        assert play(MissionSim(probs=sure), baseline_action, seed)["score"] == 40


def test_masks_and_random_policy_end():
    env = MissionSim()
    rng = np.random.default_rng(0)
    for seed in range(50):
        env.reset(seed=seed)
        for _ in range(1000):
            m = env.action_masks()
            assert m[RETURN] and m.any()
            _, _, term, trunc, _ = env.step(int(rng.choice(np.flatnonzero(m))))
            if term or trunc:
                break
        else:
            raise AssertionError("끝나지 않음")


if __name__ == "__main__":
    for name, f in list(globals().items()):
        if name.startswith("test_"):
            f()
            print("ok", name)
