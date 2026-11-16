"""Unstick controller_manager when it never receives /robot_description (isaac_mission.sh runs it).

Symptom (2026-11-16, 4 launches in a row): ros2_control_node logs "Waiting for data on 'robot_description'"
forever although robot_state_publisher is up, QoS matches (reliable + transient_local) and a fresh subscriber
gets the URDF in ~1 s. The spawner then dies after its 120 s timeout -> no joint_states / odom / TF.
Republishing the same URDF from a new node made controller_manager initialize at once.

Waits for a new ros2_control_node (so isaac_sim.sh's "other nodes on this domain" check is not tripped),
then for /controller_manager/list_controllers (it only appears after the resource manager is initialized).
If it is not there after KICK_S, republishes /robot_description, up to TRIES times. Exits 0 when the
service is up, 1 otherwise. Must run well inside the spawner's 120 s timeout.
"""
import subprocess
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

KICK_S = 25.0  # normal start brings the service up in a few seconds
TRIES = 3


def cm_pids():
    r = subprocess.run(["pgrep", "-f", "lib/controller_manager/ros2_control_node"], capture_output=True, text=True)
    return set(r.stdout.split())


# a new one: a previous run's node may still be up until isaac_sim.sh cleans it up
old_pids = cm_pids()
while not cm_pids() - old_pids:
    time.sleep(1.0)

rclpy.init()
node = rclpy.create_node("cm_kick")
log = node.get_logger()
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
urdf = []
node.create_subscription(String, "/robot_description", urdf.append, qos)
pub = node.create_publisher(String, "/robot_description", qos)
cli = node.create_client(ListControllers, "/controller_manager/list_controllers")

ok = False
for n in range(TRIES + 1):
    t0 = time.time()
    while time.time() - t0 < KICK_S and not cli.service_is_ready():
        rclpy.spin_once(node, timeout_sec=0.5)
    if cli.service_is_ready():
        ok = True
        break
    if n == TRIES:
        break
    if not urdf:
        log.warning("controller_manager not up and no /robot_description seen yet - waiting")
        continue
    log.warning(f"controller_manager not up after {KICK_S:.0f} s - republishing /robot_description ({n + 1}/{TRIES})")
    pub.publish(urdf[-1])

log.info("controller_manager up" if ok else "controller_manager still not up - check ros2_control_node log")
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
