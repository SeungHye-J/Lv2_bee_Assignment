"""컨테이너(ROS)와 파일로 주고받기 — 시뮬레이터 창들이 같이 쓴다 (lv2_module6/debug)
  읽기: debug/output/topic/<토픽>/message  (faker monitor_manager가 쓴 마지막 메시지 JSON)
  쓰기: debug/input/topic/<토픽>/message  (faker input_manager가 그 토픽으로 발행)
"""
import json
import math
import os
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEBUG = HERE.parent / 'debug'
TOPIC_DIR = DEBUG / 'output' / 'topic'
INPUT_DIR = DEBUG / 'input' / 'topic'
LIVE_DIR = DEBUG / 'input' / 'hardware' / 'camera'
WHEEL_SEPARATION = 0.287


def read_msg(topic, _cache={}):
    """debug/output/topic/<토픽>/message → (data, 몇 초 전) · 없으면 (None, None).
    같은 mtime이면 파싱 결과 재사용 (diagnostics_agg 같은 대용량 매 틱 파싱 방지) — 반환값 변경 금지"""
    path = TOPIC_DIR / topic.strip('/') / 'message'
    try:
        st = path.stat()
        hit = _cache.get(topic)
        if hit is not None and hit[0] == (st.st_mtime_ns, st.st_size):
            return hit[1], time.time() - st.st_mtime
        msg = json.loads(path.read_text())  # NaN(effort 등)도 읽힘
        data = msg.get('data')
        _cache[topic] = ((st.st_mtime_ns, st.st_size), data)
        if len(_cache) > 64:  # 토픽 수십 개 수준 — 무한 증가 방지
            _cache.pop(next(iter(_cache)))
        return data, time.time() - st.st_mtime
    except (OSError, ValueError):
        return None, None


def write_atomic(path, write):
    """임시 파일에 쓰고 rename — 읽는 쪽(fake_camera·input_manager)이 반쯤 쓰인 파일을 보지 않게"""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name('.tmp-' + path.name)
    write(tmp)
    os.replace(tmp, path)


def send(topic, msg_type, data):
    """debug/input/topic/<토픽>/message → input_manager가 발행"""
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


def current_joints(sim):
    """지금 관절값 dict + 바퀴 속도 [rad/s] — 플랜트가 돌면 플랜트 값, 아니면 /joint_states"""
    if sim is not None and sim.plant.alive:
        with sim.plant.lock:
            wheel = list(sim.plant.wheel_vel)
        return sim.plant.pose()[3], wheel
    js, _ = read_msg('joint_states')
    if not js:
        return {}, [0.0, 0.0]
    q = {n: p for n, p in zip(js['name'], js['position']) if isinstance(p, float) and math.isfinite(p)}
    vel = dict(zip(js['name'], js['velocity'])) if js.get('velocity') else {}
    return q, [vel.get('wheel_left_joint', 0.0), vel.get('wheel_right_joint', 0.0)]


def read_service_list():
    """debug/service/list.json → (data, 몇 초 전) · 없으면 (None, None)"""
    path = DEBUG / 'service' / 'list.json'
    try:
        return json.loads(path.read_text()), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None


def read_action_list():
    """debug/action/list.json → (data, 몇 초 전) · 없으면 (None, None)"""
    path = DEBUG / 'action' / 'list.json'
    try:
        return json.loads(path.read_text()), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None


def read_text(path):
    """debug/service|action/<이름>/type|info 같은 텍스트 파일 읽기"""
    try:
        p = path if isinstance(path, Path) else Path(path)
        return p.read_text()
    except OSError:
        return None


def read_interface_list():
    """debug/interface/list → (text, 몇 초 전) · 없으면 (None, None)"""
    path = DEBUG / 'interface' / 'list'
    try:
        return path.read_text(), time.time() - path.stat().st_mtime
    except OSError:
        return None, None


def read_topic_list():
    """debug/output/topic/list.json → (data, 몇 초 전) · 없으면 (None, None)"""
    path = TOPIC_DIR / 'list.json'
    try:
        return json.loads(path.read_text()), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None


def _iface_rel(type_str):
    """'ml/srv/ScoreGrasps' → 'srv/ml/ScoreGrasps' (종류별 폴더뎁스)"""
    parts = type_str.strip().strip('/').split('/')
    if len(parts) != 3:
        return None
    pkg, kind, name = parts
    if kind not in ('msg', 'srv', 'action') or not pkg or not name:
        return None
    return f'{kind}/{pkg}/{name}'


def request_interface_info(type_str):
    """debug/interface/<종류>/<패키지>/<이름>/request 쓰기 → service_action_monitor가 옆에 info 생성"""
    rel = _iface_rel(type_str)
    if rel is None:
        raise ValueError(f'타입 형식 오류 (pkg/msg|srv|action/Name): {type_str}')
    write_atomic(DEBUG / 'interface' / rel / 'request',
                 lambda p: p.write_text(json.dumps({'type': type_str.strip()}, ensure_ascii=False)))


def read_interface_info(type_str):
    """debug/interface/<종류>/<패키지>/<이름>/info → (text, 몇 초 전) · 없으면 (None, None)"""
    rel = _iface_rel(type_str)
    paths = []
    if rel is not None:
        paths.append(DEBUG / 'interface' / rel / 'info')
    # 구 레이아웃 호환 (interface/<pkg>/<kind>/<Name>/info)
    paths.append(DEBUG / 'interface' / type_str.strip().strip('/') / 'info')
    for path in paths:
        try:
            return path.read_text(), time.time() - path.stat().st_mtime
        except OSError:
            continue
    return None, None


_svc_seq = 0


def _next_id(prefix):
    global _svc_seq
    _svc_seq += 1
    return f'{prefix}-{time.strftime("%H%M%S")}-{_svc_seq % 1000:03d}'


def send_service_request(service, data, request_id=None):
    """debug/service/<이름>/request 쓰기 → service_action_input이 호출. request_id(전문번호) 반환"""
    if request_id is None:
        request_id = _next_id('q')
    payload = {'request_id': request_id, 'data': data}
    write_atomic(DEBUG / 'service' / service.strip('/') / 'request',
                 lambda p: p.write_text(json.dumps(payload, ensure_ascii=False)))
    return request_id


def read_service_response(service):
    """debug/service/<이름>/response → (dict, 몇 초 전) · 없으면 (None, None)"""
    path = DEBUG / 'service' / service.strip('/') / 'response'
    try:
        return json.loads(path.read_text()), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None


def send_action_goal(action, data, goal_id=None):
    """debug/action/<이름>/goal 쓰기 → service_action_input이 전송. goal_id 반환"""
    if goal_id is None:
        goal_id = _next_id('g')
    payload = {'goal_id': goal_id, 'data': data}
    write_atomic(DEBUG / 'action' / action.strip('/') / 'goal',
                 lambda p: p.write_text(json.dumps(payload, ensure_ascii=False)))
    return goal_id


def cancel_action_goal(action, goal_id):
    """debug/action/<이름>/cancel 쓰기 → 진행 중 goal 취소"""
    write_atomic(DEBUG / 'action' / action.strip('/') / 'cancel',
                 lambda p: p.write_text(json.dumps({'goal_id': goal_id}, ensure_ascii=False)))


def read_action_file(action, name):
    """debug/action/<이름>/accept|feedback|result → (dict, 몇 초 전)"""
    path = DEBUG / 'action' / action.strip('/') / name
    try:
        return json.loads(path.read_text()), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None
