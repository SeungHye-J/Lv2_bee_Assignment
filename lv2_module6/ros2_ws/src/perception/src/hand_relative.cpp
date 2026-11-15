// hand_relative C++판 — 설명은 include/perception/hand_relative.hpp · 파이썬판 scripts/hand_relative.py와 한 줄씩 같은 흐름
#include "perception/hand_relative.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer_interface.hpp"

namespace perception {

namespace hg = hand_geom;
using perception::msg::HandRelative;
using perception::msg::HandTarget;
using perception::msg::TrackedObjectArray;

namespace {

// 적재함 칸 가운데 (base_link, 칸 바닥 높이) — isaac/rl/scripts/slot_table.py SLOTS와 같음 (URDF basket_slot_0~11)
const std::vector<hg::V3> SLOTS = [] {
  std::vector<hg::V3> s;
  for (double x : {-0.16984, -0.10484, -0.03984})
    for (double y : {-0.0975, -0.0325, 0.0325, 0.0975}) s.push_back({x, y, 0.09125});
  return s;
}();

std::string detail(const perception::msg::TrackedObject & o, const std::string & key) {
  for (const auto & kv : o.details)
    if (kv.key == key) return kv.value;
  return "";
}

// 파이썬 int(s) if s.lstrip("-").isdigit() else -1 — '-'가 둘 이상(파이썬은 예외로 노드가 죽음)이나 넘침은 -1
int tag_of(const std::string & s) {
  const size_t neg = s.find_first_not_of('-');
  if (neg == std::string::npos || neg > 1) return -1;
  if (!std::all_of(s.begin() + neg, s.end(), [](unsigned char c) { return std::isdigit(c); })) return -1;
  try {
    return std::stoi(s);
  } catch (...) {
    return -1;
  }
}

double yaw_of_quat(const geometry_msgs::msg::Quaternion & q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

void set(geometry_msgs::msg::Vector3 & v, const hg::V3 & a) { v.x = a[0], v.y = a[1], v.z = a[2]; }

}  // namespace

HandRelativeNode::HandRelativeNode(const rclcpp::NodeOptions & options) : Node("hand_relative", options) {
  base_ = declare_parameter<std::string>("base_frame", "base_link");  // 시뮬 정책 입력 좌표 (isaac/rl/bee_isaac/tasks: base_link)
  ee_ = declare_parameter<std::string>("ee_frame", "end_effector_link");
  map_ = declare_parameter<std::string>("map_frame", "map");
  const double rate = declare_parameter<double>("rate_hz", 20.0);  // 발행 주기 (slot · 들고 있는 cell·puck 검출)
  max_age_ = declare_parameter<double>("max_age_s", 0.45);  // 이보다 오래된 검출은 안 씀 (rl_arm_correction max_cam_age_s, 시뮬 meas_age_max)
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_, *this);  // 따로 도는 스레드 — 영상 시각 TF를 기다리는 동안에도 받음
  target_.kind = "none";
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  target_sub_ = create_subscription<HandTarget>("/hand_relative/target", latched,
                                                [this](HandTarget::ConstSharedPtr m) { on_target(*m); });
  dets_sub_ = create_subscription<TrackedObjectArray>("/perception/gripper_cam/detections", 5,
                                                      [this](TrackedObjectArray::ConstSharedPtr m) { on_dets(*m); });
  pub_ = create_publisher<HandRelative>("/hand_relative", 10);
  timer_ = create_timer(std::chrono::duration<double>(1.0 / rate), [this]() { on_timer(); });
  diag_ = std::make_unique<diagnostic_updater::Updater>(this);  // /diagnostics "hand_relative: 상태" (1초마다) → aggregator 인지
  diag_->setHardwareID("none");
  diag_->add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) { on_diag(s); });
}

void HandRelativeNode::on_diag(diagnostic_updater::DiagnosticStatusWrapper & s) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  const std::string & kind = target_.kind;
  std::optional<double> age;
  if (last_valid_) age = (now() - *last_valid_).seconds();
  if (kind == "none")
    s.summary(DiagnosticStatus::OK, "대기 (target 없음)");
  else if (!age || *age > max_age_)
    s.summary(DiagnosticStatus::WARN, kind + ": 유효한 상대값 없음 (검출·TF 확인)");
  else
    s.summary(DiagnosticStatus::OK, kind + " 발행 중");
  s.add("target", kind);
  s.add("보낸 수", std::to_string(sent_));
  char buf[32];
  if (age) std::snprintf(buf, sizeof(buf), "%.2f", *age);
  s.add("마지막 유효 (s 전)", age ? std::string(buf) : std::string("-"));
}

void HandRelativeNode::publish(const HandRelative & m) {
  pub_->publish(m);
  ++sent_;
  if (m.valid) last_valid_ = now();
}

void HandRelativeNode::on_target(const HandTarget & msg) {
  target_ = msg;
  held_.reset();
  RCLCPP_INFO(get_logger(), "target %s puck_tag %d box_tag %d slot %d", msg.kind.c_str(), msg.puck_tag, msg.box_tag, msg.slot);
}

std::optional<std::pair<HandRelativeNode::Pose, HandRelativeNode::Pose>> HandRelativeNode::frames(
  const builtin_interfaces::msg::Time * stamp) {
  std::array<Pose, 2> out;
  const std::array<std::pair<const std::string *, const std::string *>, 2> pairs{{{&map_, &base_}, {&base_, &ee_}}};
  for (size_t i = 0; i < 2; ++i) {
    std::optional<geometry_msgs::msg::TransformStamped> tr;
    if (stamp != nullptr) {
      try {
        tr = tf_->lookupTransform(*pairs[i].first, *pairs[i].second, tf2_ros::fromMsg(*stamp),
                                   tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException &) {  // 그 시각 없음 → 최신으로
      }
    }
    if (!tr) {
      try {
        tr = tf_->lookupTransform(*pairs[i].first, *pairs[i].second, tf2::TimePointZero);
      } catch (const tf2::TransformException &) {  // TF가 아직 없음
        return std::nullopt;
      }
    }
    const auto & p = tr->transform.translation;
    const auto & q = tr->transform.rotation;
    out[i] = {{p.x, p.y, p.z}, hg::quat_to_rot(q.x, q.y, q.z, q.w)};
  }
  return std::make_pair(out[0], out[1]);
}

HandRelative HandRelativeNode::base_msg(const builtin_interfaces::msg::Time * stamp, const std::string & kind) const {
  HandRelative m;
  m.header.frame_id = base_;
  m.kind = kind;
  m.valid = false;
  if (stamp != nullptr) m.header.stamp = *stamp;
  m.axis.z = 1.0;
  return m;
}

void HandRelativeNode::on_timer() {
  const auto & t = target_;
  if (t.kind == "puck" || t.kind == "cell") {
    publish_held();
    return;
  }
  if (t.kind != "slot" && t.kind != "slot_puck") return;
  const auto fr = frames(nullptr);
  const builtin_interfaces::msg::Time stamp = now();
  auto m = base_msg(&stamp, t.kind);
  if (fr && 0 <= t.slot && t.slot < static_cast<int>(SLOTS.size())) {
    const auto & [ee_t, ee_R] = fr->second;
    set(m.rel, t.kind == "slot" ? hg::sub(SLOTS[t.slot], hg::bottom_in_hand(ee_t, ee_R)) : hg::slot_puck_rel(SLOTS[t.slot], ee_t));
    m.valid = true;
  }
  publish(m);
}

// 마지막 검출을 지금 손 기준으로 · 없거나 max_age_s보다 오래되면 보내지 않음 (on_dets가 invalid를 이미 보냄)
void HandRelativeNode::publish_held() {
  if (!held_) return;
  const HeldDet h = *held_;
  const double age = (now() - rclcpp::Time(h.stamp)).seconds();
  const auto fr = frames(nullptr);
  auto m = base_msg(&h.stamp, h.held.kind);
  m.age = static_cast<float>(std::max(age, 0.0));
  if (!fr || age > max_age_) {
    held_.reset();
    publish(m);
    return;
  }
  const auto & [base_t, base_R] = fr->first;
  const auto & [ee_t, ee_R] = fr->second;
  const auto r = hg::held_relative(h.held, base_t, base_R, ee_t, ee_R);
  set(m.rel, r.rel);
  m.face_yaw = static_cast<float>(r.face_yaw);
  set(m.axis, r.axis);
  m.is_box = h.held.is_box;
  if (h.held.kind == "puck") {  // 계획이 겨냥한 퍽 − 지금 손 (HandTarget.point, map)
    const auto & p = target_.point;
    set(m.insp_rel, hg::sub(hg::to_base(hg::aimed_centre({p.x, p.y, p.z}), base_t, base_R), ee_t));
  }
  if (h.nb) {
    m.neighbour = true;
    set(m.neighbour_rel, *h.nb);
  }
  m.valid = true;
  publish(m);
}

void HandRelativeNode::on_dets(const TrackedObjectArray & msg) {
  const auto & t = target_;
  if (t.kind != "puck" && t.kind != "cell") return;
  auto m = base_msg(&msg.header.stamp, t.kind);
  const auto fr = frames(&msg.header.stamp);
  const double age = (now() - rclcpp::Time(msg.header.stamp)).seconds();
  m.age = static_cast<float>(std::max(age, 0.0));
  if (!fr || age > max_age_) {  // 못 쓰는 검출: 들던 값이 있으면 그대로 (타이머가 max_age_s로 끝냄)
    if (!held_) publish(m);
    return;
  }
  const auto & [base_t, base_R] = fr->first;
  const auto & [ee_t, ee_R] = fr->second;
  if (t.kind == "cell") {
    std::optional<hg::V3> box;
    for (const auto & o : msg.objects)
      if (o.class_name == "location_box" && detail(o, "tag") == std::to_string(t.box_tag)) {
        const auto & p = o.pose.pose.position;
        box = hg::V3{p.x, p.y, p.z};
      }
    const hg::V3 pt{t.point.x, t.point.y, t.point.z};
    std::optional<hg::V3> plan;
    if (t.box_tag >= 0) plan = hg::V3{t.box_point.x, t.box_point.y, t.box_point.z};
    hg::Held held;  // 상자를 못 봐도 계획 칸으로 (손 위치는 TF로 앎)
    held.kind = "cell";
    held.c_map = hg::cell_now(pt, plan, box);
    held_ = HeldDet{msg.header.stamp, held, std::nullopt};
    publish_held();
    return;
  }
  std::vector<hg::Puck> pucks;
  std::vector<hg::Held> helds;
  for (const auto & o : msg.objects) {
    if (o.class_name != "puck" && o.class_name != "puck_lying") continue;
    const bool standing = o.class_name == "puck";
    const auto & p = o.pose.pose.position;
    const hg::V3 c_map = hg::puck_centre({p.x, p.y, p.z}, standing);
    hg::Held h;
    h.kind = "puck";
    h.c_map = c_map;
    h.standing = standing;
    if (!detail(o, "yaw_period").empty()) h.yaw_map = yaw_of_quat(o.pose.pose.orientation);
    h.is_box = detail(o, "shape") == "box";
    pucks.push_back({tag_of(detail(o, "tag")), hg::to_base(c_map, base_t, base_R)});
    helds.push_back(h);
  }
  const auto k = hg::pick_target(pucks, ee_t, t.puck_tag);
  if (!k) {  // 목표 퍽이 이 영상에 없음 (태그가 화면 밖 · 너무 가까워 위치 없음) → 들던 값은 max_age_s까지 계속 보냄
    if (!held_) publish(m);
    return;
  }
  held_ = HeldDet{msg.header.stamp, helds[*k], hg::neighbour(pucks, *k)};
  publish_held();
}

}  // namespace perception
