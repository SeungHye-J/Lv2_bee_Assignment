#!/usr/bin/env python3
"""Show rl_arm_correction as coloured ghost robots in RViz (moveit_rviz_plugin/RobotState displays).

  ref     (blue)   = JTC target = MoveIt plan before correction
  target  (green)  = ref + delta = where the measured robot should settle (twist_state: true)
  cmd     (orange) = ref + corr + delta + grav = command actually sent to the servos

Started by rviz.sh (ghost.rviz already has the three displays).
"""
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayRobotState, ObjectColor
from ml.msg import ArmCorrectionDebug

ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']  # = controllers.yaml rl_arm_correction.joints
ARM_ROOT = 'shoulder_link'  # child of joint1: ghosts draw this link and everything below it
COLORS = {'ref': (0.2, 0.5, 1.0), 'target': (0.2, 0.9, 0.3), 'cmd': (1.0, 0.55, 0.1)}


class Ghost(Node):
    def __init__(self):
        super().__init__('correction_ghost')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.on_urdf, latched)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(ArmCorrectionDebug, '/rl_arm_correction/debug', self.on_dbg, 10)
        self.pub = {g: self.create_publisher(DisplayRobotState, f'/viz/{g}/robot_state', 10) for g in COLORS}
        self.links = []
        self.others = {}  # wheels, gripper: copied from real robot

    def on_urdf(self, msg):
        root = ET.fromstring(msg.data)
        parent = {j.find('child').get('link'): j.find('parent').get('link') for j in root.findall('joint')}  # top level only: <ros2_control> joints have no parent/child

        def in_arm(link):
            while link and link != ARM_ROOT:
                link = parent.get(link)
            return link == ARM_ROOT
        self.links = [(l.get('name'), in_arm(l.get('name'))) for l in root.findall('link')]

    def on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n not in ARM:
                self.others[n] = p

    def on_dbg(self, m):
        q = {'ref': [m.ref[i] for i in range(5)],
             'target': [m.ref[i] + m.delta[i] for i in range(5)],
             'cmd': [m.ref[i] + m.corr[i] + m.delta[i] + m.grav[i] for i in range(5)]}
        for g, (r, gr, b) in COLORS.items():
            s = DisplayRobotState()
            s.state.joint_state.header.stamp = m.header.stamp
            s.state.joint_state.name = ARM + list(self.others)
            s.state.joint_state.position = q[g] + list(self.others.values())
            # alpha 0 hides the mobile base (RobotState multiplies highlight alpha by Robot Alpha)
            s.highlight_links = [ObjectColor(id=l, color=ColorRGBA(r=r, g=gr, b=b, a=1.0 if arm else 0.0))
                                 for l, arm in self.links]
            self.pub[g].publish(s)


def main():
    rclpy.init()
    n = Ghost()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
