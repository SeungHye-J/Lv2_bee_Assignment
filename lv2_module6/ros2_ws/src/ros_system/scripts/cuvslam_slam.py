#!/usr/bin/env python3
"""cuvslam_slam — cuVSLAM(NVIDIA, GPU) 시각 SLAM — 위치 담당 (지도는 nvblox → nvblox_map, ros_system.launch.py slam_nodes)

  루프 클로저 켜고 TF map→odom · /cuvslam/path + 시각 오도메트리 /cuvslam/odom(속도) → EKF가 바퀴·IMU와 합침
  publish_map_tf:=false면 시각 오도메트리만 (map→odom을 다른 곳이 낼 때)

  입력: pole_front_cam 컬러 + 정렬 depth + camera_info (RGBD 모드 — D435는 IMU가 없어 VIO 불가)
  map = 출발할 때의 base_footprint (기존 SLAM과 같음), x·y·yaw만 (기존 SLAM Reg/Force3DoF와 같음 — 경기장 바닥은 평평)
  카메라에서 min_depth 안 픽셀은 가림 — 화면 가운데 찍히는 자기 팔이 로봇과 같이 움직여 회전을 덜 잡게 함 (기존 SLAM Vis/MinDepth와 같은 이유)
  설치 (Jetson Orin + JetPack 7 — 자세히: documents/jetson-orin-nano/CUVSLAM.md): 배포 휠(cu13 aarch64)은 Thor 전용(SVE)이라 Orin에선 Illegal instruction → 소스 빌드
    sudo apt install cuda-nvcc-13-2 cuda-cudart-dev-13-2 libcublas-dev-13-2 libcusolver-dev-13-2 libcusparse-dev-13-2 \\
                     libnvjitlink-dev-13-2 cuda-nvrtc-dev-13-2 cuda-driver-dev-13-2
    git clone --branch v17.0.1 https://github.com/NVlabs/PyCuVSLAM.git ~/cuvslam/src
    PATH=/usr/local/cuda/bin:$PATH CUVSLAM_SRC_DIR=~/cuvslam/src CUVSLAM_DST_DIR=~/cuvslam/build \\
      ~/cuvslam/src/build_release.sh --build_lib --cuda_arch=87 --jobs=4
    CUVSLAM_BUILD_DIR=~/cuvslam/build pip install --user --break-system-packages ~/cuvslam/src/python/
"""
import math
import os

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def mat(q, t):
    """쿼터니언(x, y, z, w) · 이동 → 4x4"""
    x, y, z, w = q
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = t
    return m


def quat(r):
    """3x3 회전 → 쿼터니언 [x, y, z, w] (mat의 역)"""
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return [x, y, z, w]


def planar(m):
    """4x4 → (x, y, yaw) — z·roll·pitch는 버림"""
    return m[0, 3], m[1, 3], math.atan2(m[1, 0], m[0, 0])


def base_pose(base_from_cam, world_from_cam):
    """cuVSLAM world = 출발 때 카메라(OpenCV 축) → 출발 때 base_footprint 기준 지금 base_footprint (x, y, yaw)
    map_from_base = B·W·B⁻¹ (B = base_from_cam)"""
    return planar(base_from_cam @ world_from_cam @ np.linalg.inv(base_from_cam))


def map_to_odom(base, odom_from_base):
    """map_from_odom = map_from_base · odom_from_base⁻¹ (평면). base = base_pose() 결과"""
    x, y, yaw = base
    ox, oy, oyaw = planar(odom_from_base)
    a = yaw - oyaw
    c, s = math.cos(a), math.sin(a)
    return x - (c * ox - s * oy), y - (s * ox + c * oy), a


def compose(a, b):
    """평면 자세 a·b (x, y, yaw)."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    yaw = ayaw + byaw
    return ax + c * bx - s * by, ay + s * bx + c * by, math.atan2(math.sin(yaw), math.cos(yaw))


def inverse(a):
    """평면 자세 역변환."""
    x, y, yaw = a
    c, s = math.cos(yaw), math.sin(yaw)
    return -c * x - s * y, s * x - c * y, -yaw


def body_twist(prev, cur, dt):
    """두 평면 자세 사이 속도를 base_footprint 기준으로 (vx, vy, vyaw) — EKF에 넣는 값"""
    dx, dy = cur[0] - prev[0], cur[1] - prev[1]
    c, s = math.cos(prev[2]), math.sin(prev[2])
    dyaw = (cur[2] - prev[2] + math.pi) % (2 * math.pi) - math.pi
    return (c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, dyaw / dt


def apply_map_correction(map_offset, corr):
    """도면 보정 조각 corr (지금 map에서 도면 물체가 compose(corr, 도면)에 보임) → 새 map_offset = corr⁻¹ ∘ map_offset
    (map 자세 = map_offset ∘ cuVSLAM 자세라, 모든 map 자세가 corr⁻¹만큼 옮겨져 도면 물체가 제자리에 옴)"""
    return compose(inverse(corr), map_offset)


class MapGate:
    """map→odom 보정 거르기 — cuVSLAM(RGB-D 한 대 · IMU 없음)이 벽 앞·빠른 회전에서 튀어도 지도 위치가 따라 튀지 않게.
    표준(Isaac ROS VIO)은 IMU 융합으로 버티고 map→odom은 루프 클로저 때만 바뀜 — 여기선 대신:
      ① 보정이 한 번에 jump_m · jump_deg 넘게 바뀌면 받지 않고 잠금(locked) — 그 뒤 이동은 EKF(바퀴+IMU)만. /initialpose로 풀림
      ② 그 밖의 보정은 초당 rate_m · rate_deg까지만 따라감 (튀는 프레임이 지도에 바로 안 들어감)
      ③ 첫 보정 뒤 settle_s 동안(출발 자리에 서서 bringup이 뜨는 중)은 거르지 않고 그대로 받음 — 그 사이 odom이 튀어도
         (11-14 실기: 바퀴 누적 엔코더로 EKF odom이 0.75 m 튐 → 바로 잠금) map은 cuVSLAM(= 출발 자리 기준)에 맞음.
         그 사이 ① 크기로 다시 맞춘 일이 있으면 resettled에 (거리 m, 각 °)를 남김 (노드가 로그)
    11-10 시뮬 기록 재생(정답을 odom으로 봄): 4.8 m 점프 → 최대 3.5 cm · 정상 실행은 그대로. 점진적 밀림(13→56 cm)은 못 고침 → VIO 필요
    크기는 '지금 로봇 자세가 map에서 얼마나 옮겨지나'로 잰다 (odom 인자, 11-15). map→odom 값끼리 비교하면 odom 원점에서 멀수록
      작은 회전 차이가 큰 이동으로 보임 — 실기 11-15: 원점에서 4.7 m, EKF·cuVSLAM yaw 11° 차 → map→odom 이동 0.9 m로 보여
      초당 2 cm로만 따라가 map 위치가 수십 cm 늦게 붙음(보정 절반이 거름). 3~4°만 달라도 0.30 m '점프'로 잠길 수 있었음."""

    def __init__(self, jump_m=0.30, jump_deg=15.0, rate_m=0.05, rate_deg=5.0, settle_s=0.0):
        self.jump_m, self.jump_rad = jump_m, math.radians(jump_deg)
        self.rate_m, self.rate_rad = rate_m, math.radians(rate_deg)
        self.settle_s = settle_s
        self.held = None   # 지금 내는 map→odom (x, y, yaw)
        self.t = None
        self.t0 = None     # 첫 보정 시각 — reset(/initialpose)으로 지우지 않음 (정착은 bringup 때 한 번)
        self.resettled = None
        self.locked = False
        self.rejected = 0

    def reset(self):
        self.held, self.t, self.locked = None, None, False

    @staticmethod
    def _compose(a, b):
        """2D 자세 a∘b — (x, y, yaw)"""
        c, s = math.cos(a[2]), math.sin(a[2])
        return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])

    @staticmethod
    def _inverse(a):
        c, s = math.cos(a[2]), math.sin(a[2])
        return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])

    def update(self, cand, t, odom=(0.0, 0.0, 0.0)):
        """후보 map→odom, 시각 t [s], 지금 odom 자세 (x, y, yaw) → (낼 map→odom, 후보를 그대로 받았는지)
        점프·속도 한계는 로봇 자세(map→odom ∘ odom)의 변화로 잰다 — odom 원점 거리에 상관없이 같은 기준"""
        if self.t0 is None:
            self.t0 = t
        if self.held is None:
            self.held, self.t = tuple(cand), t
            return self.held, True
        dt = max(t - self.t, 1e-3)
        self.t = t
        ph, pc = self._compose(self.held, odom), self._compose(cand, odom)   # 지금 보정 · 후보 보정에서의 로봇 map 자세
        d = math.hypot(pc[0] - ph[0], pc[1] - ph[1])
        r = abs(math.remainder(pc[2] - ph[2], 2 * math.pi))
        if t - self.t0 < self.settle_s:  # ③ 출발 정착 중 — 그대로 받음
            if d > self.jump_m or r > self.jump_rad:
                self.resettled = (d, math.degrees(r))
            self.held = tuple(cand)
            return self.held, True
        if self.locked or d > self.jump_m or r > self.jump_rad:
            self.locked = True
            self.rejected += 1
            return self.held, False
        k = min(1.0, self.rate_m * dt / d if d > 1e-9 else 1.0, self.rate_rad * dt / r if r > 1e-9 else 1.0)
        if k >= 1.0:
            self.held = tuple(cand)
            return self.held, True
        # 로봇 자세를 k만큼 옮기고, 그 자세가 되게 하는 map→odom으로 (회전은 로봇 자리 기준 — 원점에서 멀어도 안 휘둘림)
        p = (ph[0] + k * (pc[0] - ph[0]), ph[1] + k * (pc[1] - ph[1]), ph[2] + k * math.remainder(pc[2] - ph[2], 2 * math.pi))
        self.held = self._compose(p, self._inverse(odom))
        self.rejected += 1
        return self.held, False


def image(m, dtype, ch):
    """sensor_msgs/Image → numpy — 메시지 버퍼를 그대로 봄(복사 없음). 행 끝 여백(row padding)이 있거나 바이트 순서가 다를 때만
    연속 배열로 복사해 cuVSLAM(네이티브)에 넘김. 잘린 버퍼·잘못된 크기는 예외. 돌려준 배열에 쓰지 말 것 (다른 구독자와 버퍼 공유).
    (Codex 11-14 · #213 복사 제거와 합침 — #213식 슬라이스는 여백이 있으면 비연속 배열을 넘겼음)"""
    dt = np.dtype(dtype)
    row_bytes = m.width * ch * dt.itemsize
    if m.height <= 0 or m.width <= 0 or ch <= 0 or m.step < row_bytes:
        raise ValueError('invalid image dimensions or row step')
    data = memoryview(m.data)
    if data.nbytes < m.height * m.step:
        raise ValueError('truncated image buffer')
    wire_dt = dt.newbyteorder('>' if getattr(m, 'is_bigendian', False) else '<')
    shape = (m.height, m.width, ch) if ch > 1 else (m.height, m.width)
    strides = (m.step, ch * dt.itemsize, dt.itemsize) if ch > 1 else (m.step, dt.itemsize)
    view = np.ndarray(shape, dtype=wire_dt, buffer=data, strides=strides)
    return np.ascontiguousarray(view, dtype=dt)


def near_mask(depth, lim):
    """uint16 depth → uint8 마스크 (1 = 0 < depth ≤ lim, 추적에서 뺌) — ((depth > 0) & (depth ≤ lim)).astype(uint8)과 같은 값.
    depth − 1을 uint16으로 빼면 0은 65535로 넘어가 한 번 비교로 끝남 (임시 배열 4개 → 1개)"""
    return (depth - np.uint16(1) < lim).view(np.uint8)


class CuvslamSlam(Node):
    def __init__(self):
        super().__init__('cuvslam_slam')
        self.base = self.declare_parameter('base_frame', 'base_footprint').value
        self.odom = self.declare_parameter('odom_frame', 'odom').value
        self.map = self.declare_parameter('map_frame', 'map').value
        # true = 단독 SLAM(루프 클로저 + TF map→odom) · false = 시각 오도메트리만 (SLAM이 map→odom)
        self.publish_map_tf = bool(self.declare_parameter('publish_map_tf', True).value)
        # BEE_CUVSLAM_NO_MAP_TF=1 — 다른 위치 추정(AMCL 스캔 매칭 등)이 map→odom을 낼 때 (시뮬 비교용, 11-15)
        if os.environ.get('BEE_CUVSLAM_NO_MAP_TF') == '1':
            self.publish_map_tf = False
        # map→odom을 이만큼 미래 시각으로 냄 (0.5 s — 영상 지연 + 발행 간격 최대 0.33 s 때문에 0.2로는 Nav2 controller가 '미래 외삽'으로 목표를 중단함, 실기 2026-11-09)
        # 최신 map→odom이 방금 들어온 depth 영상보다 옛 시각이라, nvblox가 그 영상 시각으로 TF를 찾으면 '미래 외삽'으로 실패해
        # depth를 버렸음 (실기 2026-10-29: 지도가 시작 몇 프레임 뒤로 안 자람). map→odom은 천천히 바뀌어 미래로 내도 안전
        self.map_tf_tolerance = Duration(seconds=float(self.declare_parameter('map_tf_tolerance', 1.0).value))
        #   map→odom 시각을 이만큼 미래로 (AMCL transform_tolerance 기본 1.0과 같게) — 11-15: 0.5면 미션 시작(bag 녹화·팔 동작) 순간 CPU 몰림에
        #   cuVSLAM이 ~1 s 멈춰 TF가 끊김 → controller "Failed to transform end pose" (error 102)로 출발 못 함
        self.depth_scale = float(self.declare_parameter('depth_scale_factor', 1000.0).value)  # 16UC1 mm → m
        self.min_depth = float(self.declare_parameter('min_depth', 0.5).value)  # [m] 이 안 픽셀(자기 팔)은 추적에서 뺌
        # N장에 한 장만 추적 (1 = 전부 · 2 = 15 → 7.5 Hz) — CPU 줄이기 시험용 (11-15). 빠른 제자리 회전(최대 1.8 rad/s)에서
        #   추적을 잃는지 '잃음' 수로 확인한 뒤 기본값을 바꿀 것. 속도(EKF)·map→odom은 처리한 프레임 사이 간격으로 그대로 계산됨
        self.frame_skip = max(1, int(self.declare_parameter('frame_skip', 1).value))
        self.frame_n = 0
        self.near_raw = max(math.ceil(self.min_depth * self.depth_scale) - 1, 0)  # depth 원값 < min_depth·scale ⇔ ≤ 이 값 (정수)
        # SLAM 지도 키프레임 수 상한 (cuVSLAM 기본 300) — 경기장이 작아 150이면 충분, 메모리 절약 (11-15)
        self.max_map_size = int(self.declare_parameter('max_map_size', 150).value)
        # EKF에 주는 속도 분산 — 실측으로 맞출 값 (σ 0.05 m/s · 0.05 rad/s에서 시작)
        self.lin_var = float(self.declare_parameter('linear_variance', 0.0025).value)
        self.ang_var = float(self.declare_parameter('angular_variance', 0.0025).value)
        # map→odom 거르기 (MapGate) — 큰 점프는 받지 않고 잠금(/initialpose로 풀림) · 그 밖은 초당 rate까지만 따라감. map_gate:=false면 끔
        self.gate = MapGate(float(self.declare_parameter('gate_jump_m', 0.30).value),
                            float(self.declare_parameter('gate_jump_deg', 15.0).value),
                            float(self.declare_parameter('gate_rate_m', 0.05).value),
                            float(self.declare_parameter('gate_rate_deg', 5.0).value),
                            # [s] 첫 프레임 뒤 이만큼은 거르지 않음 — 출발 자리에 서서 bringup 중 (odom 시작 튐 흡수). 이 안에 주행 시작하지 말 것
                            float(self.declare_parameter('gate_settle_s', 15.0).value)) \
            if bool(self.declare_parameter('map_gate', True).value) else None
        # BEE_MAP_FREEZE=1 (시험용, 11-13): 첫 map→odom(출발 자세 ≈ 항등)에 고정 — 보정 끔, TF map = EKF odom 그대로
        if os.environ.get('BEE_MAP_FREEZE') == '1':
            self.gate = MapGate(math.inf, math.inf, 0.0, 0.0)
            self.get_logger().warning('BEE_MAP_FREEZE=1 — map→odom 보정 끔 (출발 값에 고정)')
        self.tracker = self.base_from_cam = None
        self.last_ns = 0
        self.prev = None  # (ns, (x, y, yaw)) — 직전 오도메트리 자세. 추적을 잃으면 None (cuVSLAM 좌표가 새로 시작하므로 그 사이 속도는 안 냄)
        self.ok = self.lost = 0
        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self, spin_thread=True)  # 콜백 안에서 TF를 읽어도 막히지 않게
        self.tf_pub = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.declare_parameter('odom_topic', '/cuvslam/odom').value, 10)
        self.path_pub = self.create_publisher(Path, '/cuvslam/path', 1)
        self.path = Path()
        self.path.header.frame_id = self.map
        self.info = None
        self.map_offset = (0.0, 0.0, 0.0)   # /initialpose로 맞춘 map 프레임 보정
        self.last_base = None                # 최근 SLAM map 자세 (initialpose 보정 계산용)
        self.pending_initialpose = None
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.declare_parameter('initialpose_topic', '/initialpose').value,
                                 self.on_initialpose, 10)
        # 도면 보정 → 로봇 위치 (mission_manager arena_corr_to_localization, 11-15): [dx, dy, dth] = 보정 조각
        #   (지금 map에서 도면 물체가 apply_corr(조각, 도면) 자리에 보임) → map 기준을 조각⁻¹만큼 옮김. MapGate가 초당 rate로 따라감
        from std_msgs.msg import Float64MultiArray
        self.create_subscription(Float64MultiArray, self.declare_parameter('map_correction_topic', '/localization/map_correction').value,
                                 self.on_map_correction, 10)
        self.create_subscription(CameraInfo, self.declare_parameter('camera_info', '/pole_front_cam/color/camera_info').value,
                                 self.on_info, qos_profile_sensor_data)
        color = Subscriber(self, Image, self.declare_parameter('rgb', '/pole_front_cam/color/image_raw').value,
                           qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, self.declare_parameter('depth', '/pole_front_cam/aligned_depth_to_color/image_raw').value,
                           qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([color, depth], 10, 0.02)
        self.sync.registerCallback(self.on_rgbd)
        self.create_timer(10.0, lambda: self.get_logger().info(
            f'추적 {self.ok} · 잃음 {self.lost}' + (f' · 경로 {len(self.path.poses)}' if self.publish_map_tf else '')
            + (f' · 보정 거름 {self.gate.rejected}' + (' · 잠금' if self.gate.locked else '') if self.gate else '')))
        # BEE_CUVSLAM_VIS=1 (보기용, 11-15): 특징점·랜드마크·포즈 그래프를 RViz용으로 냄 — isaac_ros_visual_slam의 visual_slam/vis/*에 해당.
        #   트래커에 export를 켜야 해서 시작 때만 정함 · GPU→CPU 복사가 늘어 실기(Orin)에서는 끔
        self.vis = os.environ.get('BEE_CUVSLAM_VIS') == '1'
        self.last_gray = None
        if self.vis:
            self.vis_obs = self.create_publisher(Image, '/cuvslam/vis/observations_image', 1)
            self.vis_lm = self.create_publisher(PointCloud2, '/cuvslam/vis/landmarks_cloud', 1)
            self.vis_map = self.create_publisher(PointCloud2, '/cuvslam/vis/map_landmarks_cloud', 1)
            self.vis_loop = self.create_publisher(PointCloud2, '/cuvslam/vis/loop_closure_cloud', 1)
            self.vis_graph = self.create_publisher(MarkerArray, '/cuvslam/vis/pose_graph', 1)
            self.create_timer(1.0 / float(self.declare_parameter('vis_rate_hz', 2.0).value), self.publish_vis)
            self.get_logger().info('BEE_CUVSLAM_VIS=1 — /cuvslam/vis/* 냄 (특징점 영상 · 랜드마크 · 포즈 그래프)')

    def on_info(self, m):
        self.info = m

    def on_initialpose(self, m):
        """RViz 2D Pose Estimate / CLI initialpose — map 프레임에서 로봇 위치를 다시 맞춘다.
        cuVSLAM 자체를 리셋하지 않고 map→odom에 보정 offset을 건다 (추적은 계속)."""
        if m.header.frame_id not in ('', self.map):
            self.get_logger().warning(f'initialpose frame {m.header.frame_id!r} 무시 — {self.map}만 지원')
            return
        p = m.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        desired = (p.position.x, p.position.y, yaw)
        if self.last_base is None:
            self.pending_initialpose = desired
            self.get_logger().info('initialpose 대기 — 첫 추적 프레임 뒤 적용')
            return
        self.apply_initialpose(desired)

    def on_map_correction(self, m):
        if len(m.data) < 3:
            return
        self.map_offset = apply_map_correction(self.map_offset, (float(m.data[0]), float(m.data[1]), float(m.data[2])))
        self.get_logger().info(f'도면 보정 받음 → map 기준 옮김 ({m.data[0]:+.3f}, {m.data[1]:+.3f}, {math.degrees(m.data[2]):+.2f}°)')

    def apply_initialpose(self, desired):
        if self.gate is not None:
            self.gate.reset()  # 사람이 위치를 다시 맞춤 — 잠금 풀고 새 보정부터
        self.map_offset = compose(desired, inverse(self.last_base))
        self.path.poses.clear()
        self.get_logger().warning(
            f'initialpose 적용: map ({desired[0]:+.2f}, {desired[1]:+.2f}, {math.degrees(desired[2]):+.1f}°)')

    def start(self, frame):
        """첫 프레임: camera_info로 rig, URDF TF로 base_footprint←카메라. 단독 SLAM이면 루프 클로저(평면 제약) 켬"""
        import cuvslam  # 설치 안 됐으면 여기서만 실패 (SLAM 모드엔 영향 없음)
        try:
            t = self.tf_buf.lookup_transform(self.base, frame, Time()).transform
        except Exception as e:  # noqa: BLE001 — TF가 아직 안 왔으면 다음 프레임에
            self.get_logger().warning(f'TF {self.base}←{frame} 대기: {e}', throttle_duration_sec=5.0)
            return
        r = t.rotation
        self.base_from_cam = mat((r.x, r.y, r.z, r.w), (t.translation.x, t.translation.y, t.translation.z))
        k = self.info.k
        extra = {}
        # BEE_CUVSLAM_LEVEL=1 (시험, 11-15): rig를 바닥과 수평(OpenCV 축: z 앞 · x 오른쪽 · y 아래)으로 두고 카메라는 그 안에서 숙인 것으로 알림.
        #   planar_constraints는 'rig가 수평면에서 움직임'으로 자세를 고치는데, rig = 카메라(19° 숙임)면 그 면이 바닥보다 19° 기울어
        #   코너에서 돌 때부터 SLAM 랜드마크가 진행 방향으로 기울어 쌓임 (11-15 시뮬: 지도 랜드마크 주축 12° 상승 · yaw 보정 8°)
        #   이후 self.base_from_cam = base_from_rig (cuVSLAM world·자세가 rig 기준이므로 같은 변환을 그대로 씀)
        if os.environ.get('BEE_CUVSLAM_LEVEL') == '1':
            base_from_rig = np.eye(4)
            base_from_rig[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
            base_from_rig[:3, 3] = self.base_from_cam[:3, 3]
            rig_from_cam = np.linalg.inv(base_from_rig) @ self.base_from_cam
            extra['rig_from_camera'] = cuvslam.Pose(rotation=quat(rig_from_cam[:3, :3]), translation=rig_from_cam[:3, 3].tolist())
            self.base_from_cam = base_from_rig
            self.get_logger().warning('BEE_CUVSLAM_LEVEL=1 — rig 수평, 카메라는 rig 안에서 숙임 (rig_from_camera)')
        cam = cuvslam.Camera(size=(self.info.width, self.info.height), principal=(k[2], k[5]), focal=(k[0], k[4]),
                             distortion=cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole), **extra)
        rgbd = cuvslam.Tracker.OdometryRGBDSettings()
        rgbd.depth_scale_factor = self.depth_scale
        rgbd.depth_camera_id = 0
        rgbd.enable_depth_stereo_tracking = False
        odom_cfg = cuvslam.Tracker.OdometryConfig(odometry_mode=cuvslam.Tracker.OdometryMode.RGBD, async_sba=True,
                                                  rgbd_settings=rgbd)
        slam_cfg = cuvslam.Tracker.SlamConfig(planar_constraints=True, max_map_size=self.max_map_size) \
            if self.publish_map_tf else None
        if self.vis:
            odom_cfg.enable_observations_export = odom_cfg.enable_landmarks_export = True
            if slam_cfg is not None:
                slam_cfg.enable_reading_internals = True  # get_slam_landmarks · get_pose_graph
        self.tracker = cuvslam.Tracker(cuvslam.Rig([cam]), odom_cfg, slam_cfg)
        self.get_logger().info(f'cuVSLAM {cuvslam.get_version()[0]} 시작 ({"단독 SLAM" if slam_cfg else "시각 오도메트리"}) · '
                               f'{self.info.width}x{self.info.height} · 카메라 {frame}' + (f' · {self.frame_skip}장에 1장 추적' if self.frame_skip > 1 else ''))

    def on_rgbd(self, c, d):
        if self.info is None:
            return
        self.frame_n += 1
        if self.frame_skip > 1 and self.frame_n % self.frame_skip:
            return
        if self.tracker is None:
            self.start(c.header.frame_id)
            if self.tracker is None:
                return
        if (c.height, c.width) != (d.height, d.width):  # 정렬 depth는 컬러 크기여야 (RGBD 모드)
            self.get_logger().error(f'컬러 {c.width}x{c.height} ≠ depth {d.width}x{d.height}', throttle_duration_sec=5.0)
            return
        ns = Time.from_msg(c.header.stamp).nanoseconds
        if ns <= self.last_ns:  # cuVSLAM은 시각이 늘 늘어나야 함
            return
        self.last_ns = ns
        rgb = image(c, np.uint8, 3) if c.encoding in ('rgb8', 'bgr8') else image(c, np.uint8, 1)
        depth = image(d, np.uint16, 1)
        mask = near_mask(depth, self.near_raw)  # 0이 아닌 픽셀 = 추적에서 뺌
        est, slam = self.tracker.track(ns, [rgb], masks=[mask], depths=[depth])
        if self.vis:
            self.last_gray = (c.header.stamp, rgb[..., 1] if rgb.ndim == 3 else rgb)
        if est.world_from_rig is None:
            self.lost += 1
            self.prev = None
            self.get_logger().warning('추적 잃음', throttle_duration_sec=2.0)
            return
        self.ok += 1
        p = est.world_from_rig.pose
        cur = base_pose(self.base_from_cam, mat(p.rotation, p.translation))
        if self.prev is not None:
            self.publish_odom(c.header.stamp, body_twist(self.prev[1], cur, (ns - self.prev[0]) * 1e-9))
        self.prev = (ns, cur)
        self.last_base = cur
        if self.pending_initialpose is not None:
            self.apply_initialpose(self.pending_initialpose)
            self.pending_initialpose = None
        if self.publish_map_tf:
            self.publish_map(c.header.stamp, slam if slam is not None else p)  # 루프 클로저 반영된 SLAM 위치 우선

    def publish_vis(self):
        """BEE_CUVSLAM_VIS: 마지막 프레임 특징점(영상 위 점) · 그 프레임 랜드마크 · SLAM 지도 랜드마크 · 루프 클로저 랜드마크 · 포즈 그래프.
        cuVSLAM world(출발 때 카메라, OpenCV 축) 점 p → map(출발 때 base_footprint) = B·p (B = base_from_cam, base_pose와 같은 기준)"""
        if self.tracker is None or self.last_gray is None:
            return
        import cv2
        import cuvslam  # start()처럼 지역 import (모듈 맨 위에서 안 읽음)
        stamp, gray = self.last_gray
        img = cv2.cvtColor(np.ascontiguousarray(gray), cv2.COLOR_GRAY2BGR)
        obs = self.tracker.get_last_observations(0)
        for o in obs:
            cv2.circle(img, (int(o.u), int(o.v)), 3, (0, 255, 0), -1)
        cv2.putText(img, f'{len(obs)} features', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        m = Image(height=img.shape[0], width=img.shape[1], encoding='bgr8', step=img.shape[1] * 3, data=img.tobytes())
        m.header.stamp, m.header.frame_id = stamp, 'pole_front_cam_color_optical_frame'
        self.vis_obs.publish(m)
        B = self.base_from_cam
        hdr = Image().header
        hdr.stamp, hdr.frame_id = stamp, self.map

        def cloud(pub, lms):
            if not lms:
                return
            p = np.array([l.coords for l in lms], np.float64)
            p = (B[:3, :3] @ p.T).T + B[:3, 3]
            pub.publish(point_cloud2.create_cloud_xyz32(hdr, p.astype(np.float32).tolist()))
        cloud(self.vis_lm, self.tracker.get_last_landmarks())
        if self.publish_map_tf:
            L = cuvslam.Tracker.SlamDataLayer
            cloud(self.vis_map, self.tracker.get_slam_landmarks(L.Map).landmarks)
            cloud(self.vis_loop, self.tracker.get_slam_landmarks(L.LoopClosure).landmarks)
            g = self.tracker.get_pose_graph()
            xy = {n.id: base_pose(B, mat(n.node_pose.rotation, n.node_pose.translation))[:2] for n in g.nodes}
            nodes, edges = Marker(), Marker()
            for mk, ns, typ, sc, col in ((nodes, 'nodes', Marker.SPHERE_LIST, 0.04, (1.0, 0.6, 0.0)),
                                         (edges, 'edges', Marker.LINE_LIST, 0.01, (0.2, 0.8, 1.0))):
                mk.header, mk.ns, mk.type, mk.action = hdr, ns, typ, Marker.ADD
                mk.scale.x = mk.scale.y = mk.scale.z = sc
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = (*col, 1.0)
                mk.pose.orientation.w = 1.0
            nodes.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in xy.values()]
            for e in g.edges:
                if e.node_from in xy and e.node_to in xy:
                    for i in (e.node_from, e.node_to):
                        edges.points.append(Point(x=float(xy[i][0]), y=float(xy[i][1]), z=0.05))
            self.vis_graph.publish(MarkerArray(markers=[nodes, edges]))

    def publish_odom(self, stamp, v):
        """속도만 (자세는 안 씀 — cuVSLAM 좌표는 추적을 잃으면 새로 시작해 자세를 EKF에 그대로 넣으면 튐)"""
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, self.odom, self.base
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = v
        cov = [0.0] * 36
        cov[0] = cov[7] = self.lin_var
        cov[35] = self.ang_var
        cov[14] = cov[21] = cov[28] = 1e6  # 안 쓰는 축
        o.twist.covariance = cov
        self.odom_pub.publish(o)

    def publish_map(self, stamp, p):
        try:
            o = self.tf_buf.lookup_transform(self.odom, self.base, stamp).transform
        except Exception:  # noqa: BLE001 — 그 시각 odom이 없으면 이 프레임은 TF만 건너뜀 (추적은 계속)
            return
        odom_from_base = mat((o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w),
                             (o.translation.x, o.translation.y, o.translation.z))
        base_raw = base_pose(self.base_from_cam, mat(p.rotation, p.translation))
        self.last_base = base_raw
        base = compose(self.map_offset, base_raw)
        x, y, yaw = map_to_odom(base, odom_from_base)
        if self.gate is not None:
            (x, y, yaw), ok = self.gate.update((x, y, yaw), Time.from_msg(stamp).nanoseconds * 1e-9, planar(odom_from_base))
            if self.gate.resettled:
                self.get_logger().warning(
                    'map→odom 출발 정착 중 다시 맞춤 — 보정이 %.2f m · %.1f° 바뀜 (gate_settle_s 안이라 잠그지 않고 받음 · map = cuVSLAM 출발 자리 기준)'
                    % self.gate.resettled)
                self.gate.resettled = None
            if not ok:
                if self.gate.locked:
                    self.get_logger().error('map→odom 보정 잠금 — cuVSLAM 위치가 크게 튐, 바퀴+IMU로만 이어 감 '
                                            '(/initialpose로 다시 맞추면 풀림)', throttle_duration_sec=5.0)
                base = compose((x, y, yaw), planar(odom_from_base))  # 낸 보정 기준 로봇 자세 (경로 표시)
        tf = TransformStamped()
        tf.header.stamp = (Time.from_msg(stamp) + self.map_tf_tolerance).to_msg()
        tf.header.frame_id, tf.child_frame_id = self.map, self.odom
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf_pub.sendTransform(tf)
        ps = PoseStamped()
        ps.header = tf.header
        ps.pose.position.x, ps.pose.position.y = base[0], base[1]
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(base[2] / 2), math.cos(base[2] / 2)
        self.path.header.stamp = stamp
        self.path.poses.append(ps)
        del self.path.poses[:-300]  # 최근 300개(15Hz면 약 20초)만 — 11-15: 3000개는 Path 변환이 한 번 26 ms(CPU ~8%)라 줄임 (보기용)
        # 경로는 3Hz 정도로만 · 받는 쪽(RViz·Foxglove)이 있을 때만 — TUI·현장 녹화는 이 토픽을 안 받음 (map→odom은 /tf에 있음)
        if self.ok % 5 == 0 and self.path_pub.get_subscription_count() > 0:
            self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = CuvslamSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
