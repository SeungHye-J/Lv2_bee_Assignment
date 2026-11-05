"""Build cumotion/beebot.xrdf (cuMotion robot description) from the URDF collision meshes and the MoveIt SRDF.

    ~/pa-course/IsaacLab-6.1/.venv/bin/python cumotion/make_xrdf.py

Collision spheres are fitted per link with cuMotion's own generator; self-collision ignore pairs come from
ros2_ws/src/ros_system/config/moveit/beebot.srdf (the list MoveIt uses on the robot); limits are the team MoveIt
limits (3 rad/s, 5 rad/s^2), as in bee_common.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cumotion
import numpy as np
import trimesh
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bee_common as bc  # noqa: E402

HERE = Path(__file__).resolve().parent
SRDF = bc.ROOT / "ros2_ws" / "src" / "ros_system" / "config" / "moveit" / "beebot.srdf"
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
# spheres per link: enough to cover the shape at ~5 mm; the body only matters where the arm can reach it
N_SPHERES = {"base_link": 40, "pole_front_cam_link": 4, "arm_base_link": 4, "shoulder_link": 4, "upper_arm_link": 6,
             "forearm_link": 6, "wrist_link": 4, "gripper_link": 8, "finger_left_link": 3, "finger_right_link": 3,
             "gripper_cam_link": 3}
RADIUS_OFFSET = 0.002


def rpy_xyz(el):
    o = el.find("origin")
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    T = trimesh.transformations.euler_matrix(*rpy, axes="sxyz")
    T[:3, 3] = xyz
    return T


def link_mesh(link):
    parts = []
    for col in link.findall("collision"):
        g = col.find("geometry")[0]
        if g.tag == "mesh":
            m = trimesh.load(bc.URDF_PATH.parent / g.get("filename"), force="mesh")
            m.apply_scale([float(v) for v in g.get("scale", "1 1 1").split()])
        elif g.tag == "box":
            m = trimesh.creation.box([float(v) for v in g.get("size").split()])
        elif g.tag == "sphere":
            m = trimesh.creation.icosphere(radius=float(g.get("radius")))
        elif g.tag == "cylinder":
            m = trimesh.creation.cylinder(radius=float(g.get("radius")), height=float(g.get("length")))
        else:
            continue
        m.apply_transform(rpy_xyz(col))
        parts.append(m)
    return trimesh.util.concatenate(parts) if parts else None


def main():
    urdf = ET.parse(bc.URDF_PATH).getroot()
    spheres = {}
    for link in urdf.findall("link"):
        name = link.get("name")
        if name not in N_SPHERES:
            continue
        m = link_mesh(link)
        gen = cumotion.create_collision_sphere_generator([v for v in m.vertices.astype(np.float64)], [f for f in m.faces.astype(np.int32)])
        sph = gen.generate_spheres(N_SPHERES[name], RADIUS_OFFSET)
        spheres[name] = [{"center": [round(float(c), 4) for c in s.center.ravel()], "radius": round(float(s.radius), 4)} for s in sph]
        print(f"{name}: {len(sph)} spheres, mesh extents {np.round(m.extents * 1000).tolist()} mm")

    ignore = {}
    for d in ET.parse(SRDF).getroot().findall("disable_collisions"):
        a, b = d.get("link1"), d.get("link2")
        if a in N_SPHERES and b in N_SPHERES:
            ignore.setdefault(a, []).append(b)
    xrdf = {
        "format": "xrdf",
        "format_version": 1.0,
        "modifiers": [{"set_base_frame": "base_link"}],
        "default_joint_positions": dict(zip(ARM, [0.0, -1.9024, 1.344, 0.9023, 0.0])),  # SRDF stow
        "cspace": {"joint_names": ARM, "acceleration_limits": [bc.AMAX] * 5, "jerk_limits": [100.0] * 5},
        "tool_frames": ["end_effector_link"],
        "collision": {"geometry": "beebot_spheres", "buffer_distance": {k: 0.005 for k in N_SPHERES if k != "base_link"}},
        "self_collision": {"geometry": "beebot_spheres", "buffer_distance": {}, "ignore": ignore},
        "geometry": {"beebot_spheres": {"spheres": spheres}},
    }
    out = HERE / "beebot.xrdf"
    out.write_text(yaml.safe_dump(xrdf, sort_keys=False))
    rd = cumotion.load_robot_from_file(str(out), str(bc.URDF_PATH))
    print(f"wrote {out}; cuMotion loads it: {rd is not None}")


if __name__ == "__main__":
    main()
