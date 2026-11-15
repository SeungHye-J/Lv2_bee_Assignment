#!/usr/bin/env python3
"""Draw /tracked_objects pucks as bounding boxes in RViz (/viz/pucks, MarkerArray).

  box/cylinder = pose (map) + size, colour = puck colour field, faint when yaw is unknown (unit quaternion)
  arrow        = puck local +x, only when perception solved the yaw
  text         = id · posture · tag

Box = true puck size 30 x 30 x 60 mm (msg size.x/y are image-box widths, not the puck). Lying pucks (class puck_lying)
lie along their yaw: perception's yaw for a lying puck is the long axis (OBB major axis).
Started by rviz.sh (ghost.rviz has the MarkerArray display).
"""
import copy
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from perception.msg import TrackedObjectArray

RGB = {'red': (1, 0, 0), 'orange': (1, .5, 0), 'yellow': (1, 1, 0), 'green': (0, .8, 0), 'blue': (0, .3, 1),
       'purple': (.6, 0, .8), 'white': (1, 1, 1), 'black': (.15, .15, .15)}
SIDE, LONG = 0.03, 0.06  # puck [m]
H = 0.5 ** 0.5


def lay_down(q):
    """q * (pitch +90 deg): marker z (cylinder axis, long side) turns onto the puck's local +x."""
    return Quaternion(w=H * (q.w - q.y), x=H * (q.x - q.z), y=H * (q.w + q.y), z=H * (q.x + q.z))


class PuckBox(Node):
    def __init__(self):
        super().__init__('puck_box')
        self.pub = self.create_publisher(MarkerArray, '/viz/pucks', 10)
        self.create_subscription(TrackedObjectArray, '/tracked_objects', self.on_objects, 5)

    def on_objects(self, msg):
        out = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for o in msg.objects:
            if not o.class_name.startswith('puck'):  # puck / puck_lying
                continue
            d = {kv.key: kv.value for kv in o.details}
            q = o.pose.pose.orientation
            has_yaw = abs(q.w) < 0.9999  # perception sends a unit quaternion when the yaw is unknown

            lying = o.class_name == 'puck_lying' or d.get('posture') == 'lying'
            # pose z = visible TOP surface (perception depth point; puck_center shifts only x, y): measured 0.132 for a
            # standing puck on the 0.07 box (top 0.13) -> drop markers by half the puck height to the centre
            dz = (SIDE if lying else LONG) / 2

            def mk(ns, kind):
                # stamp 0 = latest TF: with the perception stamp, a map->base_footprint TF hiccup on the real robot made
                # RViz fail the transform and flash a screen-filling white box
                m = Marker(ns=ns, id=o.id, type=kind, action=Marker.ADD, header=Header(frame_id=msg.header.frame_id))
                m.pose = copy.deepcopy(o.pose.pose)  # own copy: label z / lying rotation must not move the others
                m.pose.position.z -= dz
                m.lifetime = Duration(seconds=1.0).to_msg()  # drop pucks that vanish from /tracked_objects
                return m

            box = mk('box', Marker.CYLINDER if d.get('shape') == 'cyl' else Marker.CUBE)
            box.scale.x = box.scale.y = SIDE
            box.scale.z = LONG
            if lying:
                box.pose.orientation = lay_down(q)
            box.color.r, box.color.g, box.color.b = map(float, RGB.get(o.color, (.6, .6, .6)))  # rosidl C side asserts float
            box.color.a = 0.6 if has_yaw else 0.25
            out.markers.append(box)

            if has_yaw:
                arrow = mk('yaw', Marker.ARROW)
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.06, 0.006, 0.006
                arrow.color.r = arrow.color.g = arrow.color.b = arrow.color.a = 1.0
                out.markers.append(arrow)

            text = mk('label', Marker.TEXT_VIEW_FACING)
            text.pose.position.z += dz + 0.03
            text.scale.z = 0.02
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f"{o.id} {d.get('posture', '')} t{d.get('tag', '-')}"
            out.markers.append(text)
        self.pub.publish(out)


def main():
    rclpy.init()
    n = PuckBox()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
