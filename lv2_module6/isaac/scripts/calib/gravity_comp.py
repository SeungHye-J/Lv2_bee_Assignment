#!/usr/bin/env python3
"""Gravity-twist feed-forward for the arm, outside the repo packages: publishes the joint offsets that cancel the measured joint
compliance to rl_arm_correction's ~/delta (the same input the RL policy uses), at 50 Hz.

  python3 gravity_comp.py                 # ramps in over 2 s, Ctrl-C ramps out and stops
  python3 gravity_comp.py --scale 0.5     # half strength (first try)
  python3 gravity_comp.py --k2 12.2 --k3 18.8

Per cycle: tau_i = gravity torque of everything beyond joint i at the measured angles (URDF masses, arm_model.py),
delta_i = -tau_i / k_i for joints 2 and 3 (sag_fit.py result), 0 for the others. rl_arm_correction writes
reference + corr + delta to the servos, so the hand ends up where the reference says instead of d_i = tau_i / k_i below it.
Needs arm_cam.launch.py (or the main bringup) running. rl_arm_correction limits |delta| to max_delta (0.1 rad by default); the
largest offset at full reach is about 0.125 rad, so this script raises max_delta to 0.15 first (runtime parameter, gone after a restart).
Do not run it together with rl_policy (rl_policy: true) - both publish ~/delta.
While it runs, a sag_measure.py log shows sag = bias only (about -4.9 mm) if the compensation works.
"""
import argparse
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from arm_model import Arm

parser = argparse.ArgumentParser()
parser.add_argument('--k2', type=float, default=12.2, help='joint2 stiffness [Nm/rad]')
parser.add_argument('--k3', type=float, default=18.8, help='joint3 stiffness [Nm/rad]')
parser.add_argument('--scale', type=float, default=1.0, help='fraction of the compensation (0..1)')
parser.add_argument('--ramp', type=float, default=2.0, help='s to ramp in / out')
parser.add_argument('--rate', type=float, default=50.0)
args = parser.parse_args()
MAX = 0.15  # [rad] clamp, and the max_delta we set
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

arm = Arm()
rclpy.init()
node = rclpy.create_node('gravity_comp')
latest = {}
node.create_subscription(JointState, '/joint_states', lambda m: latest.update(js=m), 10)
pub = node.create_publisher(Float64MultiArray, '/rl_arm_correction/delta', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
stop = {'flag': False}
signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))

end = time.time() + 5.0
while 'js' not in latest and time.time() < end: rclpy.spin_once(node, timeout_sec=0.1)
if 'js' not in latest: raise SystemExit('FAIL: no /joint_states')
cur = subprocess.run(['ros2', 'param', 'get', '/rl_arm_correction', 'max_delta'], capture_output=True, text=True).stdout.strip()
print('max_delta was:', cur or '(rl_arm_correction not found!)')
print(subprocess.run(['ros2', 'param', 'set', '/rl_arm_correction', 'max_delta', str(MAX)], capture_output=True, text=True).stdout.strip())


def delta():
    js = latest['js']
    idx = {n: i for i, n in enumerate(js.name)}
    q = np.array([js.position[idx[j]] for j in JOINTS])
    tau, _ = arm.gravity_and_lever(q, js.position[idx['gripper_left_joint']] if 'gripper_left_joint' in idx else 0.0)
    d = np.zeros(5)
    d[1], d[2] = -tau[1] / args.k2, -tau[2] / args.k3
    return np.clip(d, -MAX, MAX), q


t0, last_print, ramp_out = time.time(), 0.0, None
print(f'compensating (scale {args.scale}, ramp {args.ramp} s); Ctrl-C to ramp out and stop')
period = 1.0 / args.rate
while True:
    t = time.time()
    rclpy.spin_once(node, timeout_sec=0)
    if stop['flag'] and ramp_out is None: ramp_out = t
    gain = min(1.0, (t - t0) / args.ramp) if ramp_out is None else max(0.0, 1.0 - (t - ramp_out) / args.ramp)
    d, q = delta()
    pub.publish(Float64MultiArray(data=(d * args.scale * gain).tolist()))
    if t - last_print > 2.0:
        print(f'  gain {gain:.2f}  delta [deg] j2 {np.degrees(d[1]) * args.scale * gain:+.2f} j3 {np.degrees(d[2]) * args.scale * gain:+.2f}   q2 {np.degrees(q[1]):.1f} q3 {np.degrees(q[2]):.1f}')
        last_print = t
    if ramp_out is not None and gain <= 0.0: break
    time.sleep(max(0.0, period - (time.time() - t)))
pub.publish(Float64MultiArray(data=[0.0] * 5))
print('stopped, delta back to 0')
rclpy.shutdown()
