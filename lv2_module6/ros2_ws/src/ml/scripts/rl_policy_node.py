#!/usr/bin/env python3
"""🛠 rl_policy (잔차 정책 추론, 담당: 송정혁) — 팔 잔차 RL 정책(isaac/rl/bee_isaac/tasks pick_env · place_env, TorchScript)을 실기에서 돌림.

  입력: /rl_policy/context (std_msgs/String JSON — coordinator가 RL 구간 시작·끝에 냄, plan/rl_segment.hpp)
        /joint_states (q · q̇) · /rl_arm_correction/state ([reference ×5, corr ×5, Δ ×5] — 컨트롤러가 매 주기)
        /tracked_objects (pick: 대상 퍽) · TF map → base_footprint · base_footprint → end_effector_link (손끝)
  출력: /rl_arm_correction/delta (Float64MultiArray, joint1~5 [rad]) — 50 Hz (학습 정책 주기). 구간 밖에선 안 냄 → 컨트롤러가 0으로
  관측은 학습 _get_observations · demo_cycle.py pick()·place()와 같은 순서·단위 (obs_pick · obs_place). 정책 출력 a ∈ [−1, 1]:
    pick: 단계 < 3이고 이동(t_tr)이 끝난 뒤 Δ = 0.05·a, 그 뒤엔 마지막 Δ 유지 · prev_act는 매번 a
    place: 단계 < 3일 때만 추론 (Δ = 0.05·a, prev_act = a), 그 뒤엔 둘 다 유지
  카메라 관측: 학습은 gripper_cam 측정(퍽 − 손, place는 칸 − 든 퍽 바닥)을 흉내 냄 → 실기는 perception 퍽 위치 − 손끝 TF
    (place의 든 퍽 바닥 = 손끝 바로 아래로 봄). x·y만 씀 (base_footprint 기준 cm)
  enabled false · 정책 파일·torch 없음 → 아무것도 안 냄 (Δ = 0). 세우기(right) 정책은 아직 안 씀 — stand는 직교 경로 + Δ 0
"""
import json
import math
import os

import numpy as np

RESIDUAL = {"pick": 0.05, "place": 0.05}  # 학습 residual_rad
OBS_DIM = {"pick": 41, "place": 39}       # 처짐 적분까지 보는 _sag 정책 (36 + 5 · 34 + 5)
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]


def phase(t, t_ph):
    """지난 단계 끝 개수 (0 멈춤 · 1 내려감 · 2 멈춤 · 3 닫기/놓기 · 4 올라감 · 5 기다림 · 6 끝)"""
    return int(sum(t >= e for e in t_ph))


def progress(t, t_ph, ph):
    """그 단계 시작부터 지난 시간 [s], 0~2 (학습 prog — 단계 0은 구간 시작부터)"""
    start = 0.0 if ph == 0 else t_ph[min(ph, len(t_ph)) - 1]
    return min(max(t - start, 0.0), 2.0)


def onehot6(ph):
    v = np.zeros(6)
    v[min(ph, 5)] = 1.0
    return v


def box_face_err(puck_yaw, tool_yaw):
    """사각기둥: 퍽 면 yaw − 손 yaw를 ±45°로 (학습 _face_err · demo remainder(·, π/2))"""
    return math.remainder(puck_yaw - tool_yaw, math.pi / 2)


def obs_pick(q, qd, q_ref, corr, q_above, q_down, prev_act, ph, prog, cam_xy, cam_yaw, is_box):
    """pick_env.PickEnv._get_observations (+ 처짐 적분) — 41"""
    return np.concatenate([q - q_ref, q_down - q_ref, 0.1 * qd, q_ref - q_above, prev_act, onehot6(ph), [prog],
                           100.0 * np.asarray(cam_xy), [cam_yaw * float(is_box), float(is_box)], 10.0 * corr]).astype(np.float32)


def obs_place(q, qd, q_ref, corr, q_above, q_down, prev_act, ph, prog, cam_xy):
    """place_env.PlaceEnv._get_observations (+ 처짐 적분) — 39"""
    return np.concatenate([q - q_ref, q_down - q_ref, 0.1 * qd, q_ref - q_above, prev_act, onehot6(ph), [prog],
                           100.0 * np.asarray(cam_xy), 10.0 * corr]).astype(np.float32)


class Residual:
    """구간 하나의 Δ · prev_act 갱신 규칙 (demo_cycle.py와 같게)"""

    def __init__(self, skill, t_tr):
        self.skill, self.t_tr = skill, t_tr
        self.delta = np.zeros(5)
        self.prev_act = np.zeros(5)

    def wants_action(self, ph):
        return self.skill == "pick" or ph < 3  # place는 단계 3부터 추론도 멈춤

    def update(self, a, ph, t):
        a = np.clip(np.asarray(a, float), -1.0, 1.0)
        if self.skill == "pick":
            if ph < 3 and t >= self.t_tr:
                self.delta = RESIDUAL["pick"] * a
            self.prev_act = a
        elif ph < 3:
            self.delta = RESIDUAL["place"] * a
            self.prev_act = a
        return self.delta


def rotate_xy(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String
    from tf2_ros import Buffer, TransformListener
    from perception.msg import TrackedObjectArray

    class RlPolicyNode(Node):
        def __init__(self):
            super().__init__("rl_policy")
            self.declare_parameter("enabled", True)
            self.declare_parameter("rate_hz", 50.0)
            self.declare_parameter("pick_policy", "pick_residual_sag.pt")    # 파일 이름이면 share/ml/policies에서
            self.declare_parameter("place_policy", "place_residual_sag.pt")
            self.declare_parameter("device", "cpu")
            self.policies = {}
            try:
                import torch
                from ament_index_python.packages import get_package_share_directory
                self.torch = torch
                pol_dir = os.path.join(get_package_share_directory("ml"), "policies")
                for skill in ("pick", "place"):
                    name = self.get_parameter(f"{skill}_policy").value
                    path = name if os.sep in name else os.path.join(pol_dir, name)
                    m = torch.jit.load(path, map_location=self.get_parameter("device").value)
                    dim = dict(m.named_parameters())["mlp.0.weight"].shape[1]
                    if dim != OBS_DIM[skill]:
                        self.get_logger().error(f"{skill} 정책 관측 {dim}개 — {OBS_DIM[skill]}개(_sag)가 아님: 안 씀")
                        continue
                    self.policies[skill] = m.eval()
                    self.get_logger().info(f"{skill} 정책: {path}")
            except Exception as e:  # torch 없음 · 파일 없음 — Δ 0 (컨트롤러 timeout)
                self.get_logger().warn(f"정책 못 불러옴 ({e}) — Δ 0")
            self.ctx, self.res = None, None
            self.js, self.state, self.objects = {}, None, None
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            self.create_subscription(String, "/rl_policy/context", self.on_ctx, QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
            self.create_subscription(JointState, "/joint_states", self.on_js, 10)
            self.create_subscription(Float64MultiArray, "/rl_arm_correction/state", lambda m: setattr(self, "state", list(m.data)),
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(TrackedObjectArray, "/tracked_objects", lambda m: setattr(self, "objects", m),
                                     QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
            self.pub = self.create_publisher(Float64MultiArray, "/rl_arm_correction/delta",
                                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)

        def on_ctx(self, m):
            c = json.loads(m.data)
            if not c.get("skill"):
                if self.ctx:
                    self.get_logger().info(f"RL {self.ctx['skill']} 끝")
                self.ctx, self.res = None, None
                return
            self.ctx = c
            self.res = Residual(c["skill"], c["t_tr"])
            on = c["skill"] in self.policies and self.get_parameter("enabled").value
            self.get_logger().info(f"RL {c['skill']} 시작 (정책 {'켬' if on else '없음 — Δ 0'})")

        def on_js(self, m):
            self.js.update(zip(m.name, zip(m.position, m.velocity if len(m.velocity) == len(m.name) else [0.0] * len(m.name))))

        def lookup(self, target, source):
            try:
                t = self.tf.lookup_transform(target, source, rclpy.time.Time())
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                return t.transform.translation, yaw
            except Exception:
                return None, 0.0

        def cam_xy(self, c):
            """퍽(pick) · 놓을 칸(place) − 손끝, base_footprint 기준 x·y [m] · 못 구하면 None"""
            ee, _ = self.lookup("base_footprint", "end_effector_link")
            base, byaw = self.lookup("map", "base_footprint")
            if base is None:  # 지도 없이 실행 — perception은 odom 좌표를 map이라 적어 냄 (coordinator world_frame과 같게)
                base, byaw = self.lookup("odom", "base_footprint")
            if ee is None or base is None:
                return None
            if c["skill"] == "pick":
                o = next((o for o in (self.objects.objects if self.objects else []) if o.id == c["target_id"]), None)
                if o is None:
                    return None
                px, py = o.pose.pose.position.x, o.pose.pose.position.y
            else:
                px, py = c["cell"][0], c["cell"][1]
            bx, by = rotate_xy(px - base.x, py - base.y, -byaw)  # map → base_footprint
            return bx - ee.x, by - ee.y

        def puck_yaw(self, c):
            o = next((o for o in (self.objects.objects if self.objects else []) if o.id == c["target_id"]), None)
            if o is None or not any(kv.key == "yaw_period" for kv in o.details):
                return None
            q = o.pose.pose.orientation
            return 2 * math.atan2(q.z, q.w)

        def tick(self):
            c, r = self.ctx, self.res
            if c is None or c["skill"] not in self.policies or not self.get_parameter("enabled").value:
                return  # 안 냄 → 컨트롤러 delta_timeout 뒤 Δ 0
            t = self.get_clock().now().nanoseconds * 1e-9 - c["t0"]
            if t > c["t_ph"][-1] + 1.0:  # 끝 알림을 놓쳤어도 멈춤
                self.ctx = None
                return
            if self.state is None or len(self.state) < 15 or any(j not in self.js for j in ARM):
                return
            q = np.array([self.js[j][0] for j in ARM])
            qd = np.array([self.js[j][1] for j in ARM])
            q_ref, corr = np.array(self.state[0:5]), np.array(self.state[5:10])
            if np.any(np.isnan(q_ref)):
                return
            ph = phase(t, c["t_ph"])
            prog = progress(t, c["t_ph"], ph)
            cam = self.cam_xy(c)
            if cam is None:
                cam = (0.0, 0.0)  # 못 보면 0 (학습엔 없는 경우 — 위치 보정 없이 처짐·추종만)
            qa, qdn = np.array(c["q_above"]), np.array(c["q_down"])
            if r.wants_action(ph):
                if c["skill"] == "pick":
                    py = self.puck_yaw(c)
                    face = box_face_err(py, c["tool_yaw"]) if (c["is_box"] and py is not None) else 0.0
                    obs = obs_pick(q, qd, q_ref, corr, qa, qdn, r.prev_act, ph, prog, cam, face, c["is_box"])
                else:
                    obs = obs_place(q, qd, q_ref, corr, qa, qdn, r.prev_act, ph, prog, cam)
                with self.torch.inference_mode():
                    a = self.policies[c["skill"]](self.torch.from_numpy(obs)[None])[0].cpu().numpy()
                r.update(a, ph, t)
            m = Float64MultiArray()
            m.data = [float(v) for v in r.delta]
            self.pub.publish(m)

    rclpy.init()
    node = RlPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
