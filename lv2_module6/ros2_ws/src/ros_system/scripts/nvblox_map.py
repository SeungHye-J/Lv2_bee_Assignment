#!/usr/bin/env python3
"""nvblox_map — nvblox 2D 점유 격자를 /map으로 + 저장 지도 불러오기

  /map은 latched(reliable + transient_local)여야 Nav2 global_costmap static_layer(map_subscribe_transient_local) ·
  state_manager · RViz · 시뮬레이터가 그대로 받는다. nvblox ~/static_occupancy_grid는 주기적으로 나오는 보통 토픽이라
  QoS가 안 맞아 직접은 못 받음 → 마지막 격자를 rate_hz로 /map에 다시 냄 (새 격자가 왔을 때만 — static_layer가 매번 다시 그리지 않게)
  load_path가 있으면(map:=saved) 시작할 때 /nvblox_node/load_map으로 불러옴. 저장은 nvblox after_shutdown_map_save_path(끌 때 자동)
  저장 지도의 좌표 = 그때 출발 자리 — 지금 출발 자리가 같아야 맞음 (운영 규칙: 출발 구역 같은 자리·같은 방향에서 켬)
  walls_in(/arena/walls, mission_manager — 도면의 경기장 벽)이 오면 /map = 그 격자 크기(경기장 전체)에 벽 + nvblox를 겹친 것:
    매핑 처음부터 Nav2가 벽을 알아 모르는 칸으로 벽을 뚫는 경로를 안 냄 · /map 크기가 고정이라 costmap이 매번 크기를 바꾸지 않음.
    벽 칸은 늘 점유(100), 나머지는 nvblox 값 (nvblox 격자가 경기장 밖으로 나간 부분은 버림)
  speckle_min_cells(0 = 끔, 시험 값 8): nvblox 점유 칸 덩어리(8방향 이웃)가 이보다 작으면 모름(-1)으로 — depth 잡음 점. 8~30 cm 높이 층에 이만큼 작은
    진짜 물체는 없음 (상자·퍽은 8 cm 아래) · 가까운 장애물은 local costmap(카메라)·collision_monitor가 따로 봄. 0 = 끔.
    11-15 실기 지도 20261115_110106: 0.55 m 문 가운데 2~3칸 점이 문을 둘로 갈라 (각 0.25 m < 로봇 내접 지름 0.26 m) 전역 경로가 없었음
"""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


class NvbloxMap(Node):
    def __init__(self):
        super().__init__('nvblox_map')
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.load_path = self.declare_parameter('load_path', '').value
        rate = float(self.declare_parameter('rate_hz', 1.0).value)
        self.speckle = int(self.declare_parameter('speckle_min_cells', 0).value)  # 11-15 예전으로 (끔) — 시험할 때 8
        self.n_speckle = 0
        self.grid = None
        self.fresh = False
        self.n_in = self.n_out = 0
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, self.declare_parameter('map_out', '/map').value, latched)
        # best effort로 구독 — 발행 쪽이 reliable이든 best effort든 받음
        self.create_subscription(OccupancyGrid, self.declare_parameter('grid_in', '/nvblox_node/static_occupancy_grid').value,
                                 self.on_grid, qos_profile_sensor_data)
        self.walls = None
        self.create_subscription(OccupancyGrid, self.declare_parameter('walls_in', '/arena/walls').value, self.on_walls, latched)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(10.0, lambda: self.get_logger().info(
            f'격자 받음 {self.n_in} · /map 냄 {self.n_out} · 지운 잡음 점 {self.n_speckle}칸 (마지막 격자)' +
            ('' if self.grid else ' — nvblox 격자 아직 없음')))
        if self.load_path:
            self.load()

    def on_grid(self, m):
        if m.header.frame_id and m.header.frame_id != self.frame:
            self.get_logger().error(f'격자 프레임 {m.header.frame_id} ≠ {self.frame} — nvblox global_frame 확인',
                                    throttle_duration_sec=10.0)
            return
        self.grid = m
        self.fresh = True
        self.n_in += 1

    def on_walls(self, m):
        self.walls = m
        self.fresh = True
        self.get_logger().info(f'경기장 벽 격자 받음 {m.info.width}x{m.info.height} — /map에 겹침')

    def tick(self):
        if not self.fresh:
            return
        self.fresh = False
        grid = self.grid
        if grid is not None and self.speckle > 0:
            grid, self.n_speckle = despeckle(grid, self.speckle)
        out = grid if self.walls is None else overlay(self.walls, grid)
        out.header.frame_id = self.frame
        self.pub.publish(out)
        self.n_out += 1

    def load(self):
        """nvblox_node가 뜰 때까지 1 s마다 봄 (최대 60 s) — 생성자에서 기다리면 같은 프로세스의 다른 노드(py_helpers)까지 멈춤"""
        from nvblox_msgs.srv import FilePath  # nvblox 설치된 곳에서만 (launch check_slam이 확인)
        self.load_cli = self.create_client(FilePath, '/nvblox_node/load_map')
        self.load_req = FilePath.Request(file_path=self.load_path)
        self.load_wait = 0
        self.load_timer = self.create_timer(1.0, self.try_load)

    def try_load(self):
        if not self.load_cli.service_is_ready():
            self.load_wait += 1
            if self.load_wait >= 60:
                self.load_timer.cancel()
                self.get_logger().error(f'nvblox load_map 서비스 없음 — {self.load_path} 못 불러옴 (빈 지도로 시작)')
            return
        self.load_timer.cancel()
        self.load_cli.call_async(self.load_req).add_done_callback(lambda f: self.get_logger().info(
            f'지도 불러오기 {self.load_path}: {"성공" if f.result() and f.result().success else "실패 — 빈 지도로 시작"}'))


def despeckle(grid, min_cells):
    """점유(≥ 50) 덩어리 중 min_cells 칸보다 작은 것을 모름(-1)으로 바꾼 → (새 격자, 바꾼 칸 수). scipy 없으면 그대로"""
    try:
        from scipy.ndimage import label
    except ImportError:
        return grid, 0
    a = np.array(grid.data, np.int8).reshape(grid.info.height, grid.info.width)
    occ = a >= 50
    lab, _ = label(occ, structure=np.ones((3, 3)))
    small = occ & (np.bincount(lab.ravel())[lab] < min_cells)
    a[small] = -1
    m = OccupancyGrid()
    m.header = grid.header
    m.info = grid.info
    m.data = a.ravel().tolist()
    return m, int(small.sum())


def overlay(walls, grid):
    """walls 격자 크기로: 벽(100)은 그대로, 나머지는 nvblox 값 (해상도가 같다고 봄 — 둘 다 0.05 m). grid가 None이면 벽만"""
    w, h = walls.info.width, walls.info.height
    out = np.array(walls.data, np.int8).reshape(h, w)
    if grid is not None and grid.info.width and grid.info.height:
        r = walls.info.resolution
        a = np.array(grid.data, np.int8).reshape(grid.info.height, grid.info.width)
        ox = int(round((grid.info.origin.position.x - walls.info.origin.position.x) / r))
        oy = int(round((grid.info.origin.position.y - walls.info.origin.position.y) / r))
        x0, y0 = max(ox, 0), max(oy, 0)
        x1, y1 = min(ox + a.shape[1], w), min(oy + a.shape[0], h)
        if x1 > x0 and y1 > y0:
            sub = a[y0 - oy:y1 - oy, x0 - ox:x1 - ox]
            dst = out[y0:y1, x0:x1]
            np.copyto(dst, sub, where=dst != 100)
    m = OccupancyGrid()
    m.header = walls.header
    m.info = walls.info
    m.data = out.ravel().tolist()
    return m


def main():
    rclpy.init()
    node = NvbloxMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
