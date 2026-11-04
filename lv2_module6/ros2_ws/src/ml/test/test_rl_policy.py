"""rl_policy_node 관측·잔차 규칙 테스트 (ROS 통신 없이) — 학습 _get_observations · demo_cycle.py와 같은 순서·크기"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import rl_policy_node as rp  # noqa: E402

T_PH = [0.6, 1.2, 1.5, 2.0, 2.6, 2.9]  # 단계 끝 (이동+멈춤 · 내려감 · 멈춤 · 닫기 · 올라감 · 기다림)


def test_phase_progress_onehot():
    assert rp.phase(0.0, T_PH) == 0
    assert rp.phase(1.3, T_PH) == 2
    assert rp.phase(3.0, T_PH) == 6
    assert rp.progress(0.4, T_PH, 0) == pytest.approx(0.4)
    assert rp.progress(1.3, T_PH, 2) == pytest.approx(0.1)
    assert rp.progress(9.0, T_PH, 6) == 2.0  # 2초에서 자름
    assert list(rp.onehot6(6)) == [0, 0, 0, 0, 0, 1]


def test_obs_pick_layout():
    z = np.zeros(5)
    q, q_ref = np.full(5, 0.5), np.full(5, 0.4)
    q_above, q_down = np.full(5, 0.3), np.full(5, 0.6)
    corr = np.full(5, 0.01)
    o = rp.obs_pick(q, np.ones(5), q_ref, corr, q_above, q_down, z, 1, 0.25, (0.01, -0.02), 0.1, True)
    assert o.shape == (41,)
    assert o[0] == pytest.approx(0.1)          # q − q_ref
    assert o[5] == pytest.approx(0.2)          # q_down − q_ref
    assert o[10] == pytest.approx(0.1)         # 0.1·q̇
    assert o[15] == pytest.approx(0.1)         # q_ref − q_above
    assert list(o[25:31]) == [0, 1, 0, 0, 0, 0]
    assert o[31] == pytest.approx(0.25)
    assert o[32:34] == pytest.approx([1.0, -2.0])  # cm
    assert o[34] == pytest.approx(0.1) and o[35] == 1.0
    assert o[36] == pytest.approx(0.1)         # 10·corr
    cyl = rp.obs_pick(q, z, q_ref, corr, q_above, q_down, z, 1, 0.25, (0, 0), 0.1, False)
    assert cyl[34] == 0.0 and cyl[35] == 0.0   # 원기둥은 면 오차·표시 0


def test_obs_place_layout():
    z = np.zeros(5)
    o = rp.obs_place(z, z, z, np.full(5, 0.02), z, z, z, 0, 0.0, (0.003, 0.0))
    assert o.shape == (39,)
    assert o[32] == pytest.approx(0.3)
    assert o[34] == pytest.approx(0.2)


def test_box_face_err_wraps():
    assert rp.box_face_err(math.radians(50), 0.0) == pytest.approx(math.radians(-40))
    assert rp.box_face_err(math.radians(30), math.pi) == pytest.approx(math.radians(30))  # 손 ψ와 학습 yaw는 π 차이 — 주기 90°라 같음


def test_residual_rules():
    a = np.full(5, 2.0)  # [−1, 1]로 자름
    pick = rp.Residual("pick", t_tr=0.5)
    assert pick.update(a, 0, 0.2) == pytest.approx(np.zeros(5))   # 이동 중엔 Δ 안 씀
    assert list(pick.prev_act) == [1.0] * 5                         # prev_act는 매번
    assert pick.update(a, 1, 0.8) == pytest.approx(np.full(5, 0.05))
    assert pick.update(-a, 3, 1.8) == pytest.approx(np.full(5, 0.05))  # 닫은 뒤 Δ 유지
    assert list(pick.prev_act) == [-1.0] * 5
    place = rp.Residual("place", t_tr=0.0)
    assert place.wants_action(2) and not place.wants_action(3)
    place.update(a, 1, 0.8)
    place.update(-a, 4, 2.2)                                        # 단계 3부터 안 바뀜
    assert place.delta == pytest.approx(np.full(5, 0.05))
    assert list(place.prev_act) == [1.0] * 5


@pytest.mark.parametrize("skill,name", [("pick", "pick_residual_sag.pt"), ("place", "place_residual_sag.pt")])
def test_policy_files_match_obs(skill, name):
    torch = pytest.importorskip("torch")
    path = os.path.join(HERE, "..", "policies", name)
    m = torch.jit.load(path, map_location="cpu")
    assert dict(m.named_parameters())["mlp.0.weight"].shape[1] == rp.OBS_DIM[skill]
    z = np.zeros(5)
    obs = rp.obs_pick(z, z, z, z, z, z, z, 0, 0.0, (0, 0), 0.0, False) if skill == "pick" else \
        rp.obs_place(z, z, z, z, z, z, z, 0, 0.0, (0, 0))
    with torch.inference_mode():
        a = m(torch.from_numpy(obs)[None])[0].numpy()
    assert a.shape == (5,) and np.all(np.isfinite(a))
