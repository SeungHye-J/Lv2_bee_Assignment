#pragma once

// nvblox_map 격자 계산 (ROS 노드 없음 · test/test_grid_ops.cpp) — scripts/nvblox_map.py despeckle · overlay와 같은 결과 (11-16 C++판)
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace ros_system {

// 점유(≥ 50) 덩어리(8방향 이웃) 중 min_cells 칸보다 작은 것을 모름(-1)으로 → (새 격자, 바꾼 칸 수)
//   파이썬판 scipy.ndimage.label(structure 3×3) + bincount와 같음
inline nav_msgs::msg::OccupancyGrid despeckle(const nav_msgs::msg::OccupancyGrid & grid, int min_cells, long & changed) {
  nav_msgs::msg::OccupancyGrid m;
  m.header = grid.header;
  m.info = grid.info;
  m.data = grid.data;
  changed = 0;
  const long w = grid.info.width, h = grid.info.height;
  if (w <= 0 || h <= 0 || static_cast<long>(m.data.size()) != w * h) return m;
  std::vector<int32_t> lab(w * h, -1);  // -1 = 아직 안 봄 (점유 칸만)
  std::vector<long> stack, members;
  for (long i = 0; i < w * h; ++i) {
    if (m.data[i] < 50 || lab[i] >= 0) continue;
    members.clear();
    stack.assign(1, i);
    lab[i] = 1;
    while (!stack.empty()) {
      const long c = stack.back();
      stack.pop_back();
      members.push_back(c);
      const long cx = c % w, cy = c / w;
      for (long dy = -1; dy <= 1; ++dy)
        for (long dx = -1; dx <= 1; ++dx) {
          const long x = cx + dx, y = cy + dy;
          if (x < 0 || y < 0 || x >= w || y >= h) continue;
          const long n = y * w + x;
          if (lab[n] >= 0 || m.data[n] < 50) continue;
          lab[n] = 1;
          stack.push_back(n);
        }
    }
    if (static_cast<long>(members.size()) < min_cells) {
      for (long c : members) m.data[c] = -1;
      changed += static_cast<long>(members.size());
    }
  }
  return m;
}

// walls 격자 크기로: 벽(100)은 그대로, 나머지는 nvblox 값 (해상도가 같다고 봄 — 둘 다 0.05 m). grid가 nullptr이면 벽만
//   원점 차이 칸 수 = 반올림(짝수 쪽 — 파이썬 round와 같음) · nvblox 격자가 경기장 밖으로 나간 부분은 버림
inline nav_msgs::msg::OccupancyGrid overlay(const nav_msgs::msg::OccupancyGrid & walls, const nav_msgs::msg::OccupancyGrid * grid) {
  nav_msgs::msg::OccupancyGrid m;
  m.header = walls.header;
  m.info = walls.info;
  m.data = walls.data;
  const long w = walls.info.width, h = walls.info.height;
  if (grid == nullptr || !grid->info.width || !grid->info.height) return m;
  const long gw = grid->info.width, gh = grid->info.height;
  if (static_cast<long>(m.data.size()) != w * h || static_cast<long>(grid->data.size()) != gw * gh) return m;
  const double r = walls.info.resolution;
  const long ox = std::lrint(std::nearbyint((grid->info.origin.position.x - walls.info.origin.position.x) / r));
  const long oy = std::lrint(std::nearbyint((grid->info.origin.position.y - walls.info.origin.position.y) / r));
  const long x0 = std::max(ox, 0L), y0 = std::max(oy, 0L);
  const long x1 = std::min(ox + gw, w), y1 = std::min(oy + gh, h);
  if (x1 <= x0 || y1 <= y0) return m;
  for (long y = y0; y < y1; ++y)
    for (long x = x0; x < x1; ++x) {
      int8_t & d = m.data[y * w + x];
      if (d != 100) d = grid->data[(y - oy) * gw + (x - ox)];
    }
  return m;
}

}  // namespace ros_system
