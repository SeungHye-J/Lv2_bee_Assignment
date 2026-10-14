"""컨테이너(ROS)와 파일로 주고받기 — 시뮬레이터 창들이 같이 쓴다 (lv2_module6/debug)
  읽기: debug/topic/<토픽>/message  (faker monitor_manager가 쓴 마지막 메시지 JSON)
  쓰기: debug/input/<토픽>/message  (faker input_manager가 그 토픽으로 발행)
"""
import json
import math
import os
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEBUG = HERE.parent / 'debug'
TOPIC_DIR = DEBUG / 'topic'
INPUT_DIR = DEBUG / 'input'
LIVE_DIR = DEBUG / 'input-live'
SIM_DIR = DEBUG / 'sim'
WHEEL_SEPARATION = 0.287


def read_msg(topic):
    """debug/topic/<토픽>/message → (data, 몇 초 전) · 없으면 (None, None)"""
    path = TOPIC_DIR / topic.strip('/') / 'message'
    try:
        msg = json.loads(path.read_text())  # NaN(effort 등)도 읽힘
        return msg.get('data'), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None


def write_atomic(path, write):
    """임시 파일에 쓰고 rename — 읽는 쪽(fake_camera·input_manager)이 반쯤 쓰인 파일을 보지 않게"""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name('.tmp-' + path.name)
    write(tmp)
    os.replace(tmp, path)


def send(topic, msg_type, data):
    """debug/input/<토픽>/message → input_manager가 발행"""
    write_atomic(INPUT_DIR / topic.strip('/') / 'message',
                 lambda p: p.write_text(json.dumps({'type': msg_type, 'data': data}, ensure_ascii=False)))


HEADER = {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': 'base_footprint'}


def send_bool(topic, value):
    send(topic, 'std_msgs/msg/Bool', {'data': bool(value)})


def send_twist(v, w):
    send('/manual/cmd_vel', 'geometry_msgs/msg/TwistStamped',
         {'header': HEADER, 'twist': {'linear': {'x': v, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': w}}})


def send_arm(targets):
    send('/manual/arm', 'sensor_msgs/msg/JointState',
         {'header': HEADER, 'name': list(targets), 'position': list(targets.values()), 'velocity': [], 'effort': []})


def yaw_of(q):
    return math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), 1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
