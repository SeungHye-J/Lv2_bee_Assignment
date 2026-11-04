"""Stand-up + in-place regrasp through the real ROS arm path (stand_dummy.sh) — a stand-in for the coordinator skill.

  arm:     /arm_controller/follow_joint_trajectory (JTC) -> rl_arm_correction (delta 0, sag integral) -> OpenCR
  gripper: /gripper_controller/gripper_cmd (position); force cap via /tmp/opencr_grip_force (Isaac dummy only:
           the firmware has no runtime gripper current command yet)
  plan:    isaac/lab/stand_plan.py (perceived lying puck + offline IK, the right_env 'tilt' motion)

Sequence: drive straight to the dock (/cmd_vel -> diff_drive_controller, wheel /odom; stand-in for Nav2) ->
reset sag -> open -> stow to above, hold, down -> pinch (0.4 N cap) -> arc, settle -> open ->
regrasp from the measured joints (wrist kept, straight down) -> hold (sag integral) -> close (full force) -> lift.
Grasp check without a camera: the finger stops at the puck width (gripper joint near GRIP_STOP), not at 0.
"""
import json
import math
import os
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
FORCE_FILE = "/tmp/opencr_grip_force"


def dur(t):
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))


class StandDummy(Node):
    def __init__(self, plan):
        super().__init__("stand_dummy")
        self.p = plan
        self.js = {}
        self.odom = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m.pose.pose), 10)
        self.cmd = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, ParallelGripperCommand, "/gripper_controller/gripper_cmd")
        self.reset = self.create_client(Trigger, "/rl_arm_correction/reset_sag")

    def on_js(self, m):
        self.js.update(zip(m.name, m.position))

    def spin_until(self, fut, timeout=60.0):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def wait_sim(self, secs):
        """Wait secs of sim time (Isaac runs slower than real time)."""
        t0 = self.get_clock().now()
        while (self.get_clock().now() - t0).nanoseconds < secs * 1e9:
            rclpy.spin_once(self, timeout_sec=0.01)

    def q_now(self):
        return [self.js[j] for j in ARM]

    def drive(self, dist):
        """Straight ahead by dist [m] on wheel odometry (P control, 0.08 m/s cap), then stop and settle."""
        while self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        p0, q = self.odom.position, self.odom.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        done = lambda: (self.odom.position.x - p0.x) * math.cos(yaw) + (self.odom.position.y - p0.y) * math.sin(yaw)
        self.get_logger().info(f"drive: {1000 * dist:.0f} mm to the dock")
        t0 = self.get_clock().now()
        while abs(dist - done()) > 0.002 and (self.get_clock().now() - t0).nanoseconds < 20e9:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist.linear.x = max(-0.08, min(0.08, 1.5 * (dist - done())))
            self.cmd.publish(m)
            self.wait_sim(0.02)
        self.cmd.publish(TwistStamped())
        self.wait_sim(0.5)
        self.get_logger().info(f"drive: stopped {1000 * (done() - dist):+.1f} mm from the dock (odometry)")

    def follow(self, samples, label):
        """Stream joint samples (dt apart) as one trajectory, starting from the measured joints."""
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ARM
        g.trajectory.points = [JointTrajectoryPoint(positions=list(map(float, q)), time_from_start=dur((k + 1) * self.p["dt"]))
                               for k, q in enumerate(samples)]
        self.get_logger().info(f"{label}: {len(samples)} points, {len(samples) * self.p['dt']:.2f} s")
        h = self.spin_until(self.arm.send_goal_async(g))
        if not h or not h.accepted:
            raise RuntimeError(f"{label}: trajectory rejected")
        r = self.spin_until(h.get_result_async(), timeout=120.0)
        if r.result.error_code != 0:
            self.get_logger().warn(f"{label}: JTC error {r.result.error_code} {r.result.error_string}")

    def move(self, q_end, t_move, label):
        """Smooth step from the measured joints (bee_common.move_s) — the regrasp starts where the hand really is."""
        q0, n = self.q_now(), max(1, round(t_move / self.p["dt"]))
        s = lambda x: x * x * (3 - 2 * x)
        self.follow([[a + s((k + 1) / n) * (b - a) for a, b in zip(q0, q_end)] for k in range(n)], label)

    def gripper(self, pos, force=None, secs=0.0, label=""):
        if force is None:
            if os.path.exists(FORCE_FILE):
                os.remove(FORCE_FILE)
        else:
            with open(FORCE_FILE, "w") as f:
                f.write(f"{force}")
        g = ParallelGripperCommand.Goal()
        g.command.name = ["gripper_left_joint"]
        g.command.position = [float(pos)]
        self.get_logger().info(f"{label}: gripper {1000 * pos:.1f} mm, force cap {force if force is not None else 'default'}")
        self.grip.send_goal_async(g)  # do not wait for the result: a stalled close may never 'reach' the target
        self.wait_sim(secs)

    def run(self):
        p = self.p
        for c in (self.arm, self.grip):
            if not c.wait_for_server(timeout_sec=180.0):  # spawner may take up to 120 s
                raise RuntimeError(f"no action server {c._action_name}")
        while len(self.js) < 6:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.drive(p["drive_m"])
        self.get_logger().info(f"puck true {p['puck_true']}, perceived {p['puck_meas']} (m, m, deg) at the dock")
        if not self.reset.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("no /rl_arm_correction/reset_sag (ml not rebuilt?)")
        self.spin_until(self.reset.call_async(Trigger.Request()))  # skill start: sag integral from zero
        self.gripper(p["open_m"], None, 0.5, "open")
        self.follow(p["approach"], "stand: stow -> above -> down")
        self.gripper(0.0, p["pinch_force_n"], p["close_s"], "stand: pinch")
        self.follow(p["arc"], "stand: arc")
        self.wait_sim(p["settle_s"])
        self.gripper(p["open_m"], p["pinch_force_n"], p["open_s"], "stand: open")
        # regrasp: no reset_sag here (corr -> 0 would drop the hand by the sag); start from the measured joints
        self.move(p["q_grasp"], p["t_grasp"], "regrasp: straight down")
        self.wait_sim(p["hold_s"])
        self.gripper(0.0, p["grip_force_n"], p["regrasp_close_s"], "regrasp: close")
        self.move(p["q_lift"], p["t_lift"], "regrasp: lift")
        self.wait_sim(0.5)
        g = self.js["gripper_left_joint"]
        held = g > 0.5 * p["grip_stop_m"]
        self.get_logger().info(f"RESULT {'HOLDING' if held else 'EMPTY'}: gripper joint {1000 * g:.2f} mm "
                               f"(puck width stop {1000 * p['grip_stop_m']:.2f} mm, empty 0)")
        return held


def main():
    rclpy.init()
    node = StandDummy(json.load(open(os.environ.get("STAND_PLAN", "/tmp/stand_plan.json"))))
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
