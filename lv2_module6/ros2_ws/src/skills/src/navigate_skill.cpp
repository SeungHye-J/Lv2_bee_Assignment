// skill_navigate — /skills/navigate: goal.pose(map)로 Nav2 NavigateToPose (지도 위 먼 이동, 장애물 피함)
//   목표 자세 계산(물체 앞 · 좌표)은 coordinator가 함. 학습 정책 없음 (Nav2 그대로)
//   진척 감시: 남은 경로 거리가 nav_stall_s 동안 nav_stall_gain_m만큼도 안 줄면 취소하고 nav_no_progress (ProgressWatch)
//   goal.target "look" (상자 도킹 앞 주행, 11-13): 남은 거리가 nav_look_m 안이면 팔을 pose_look으로 천천히 (approach가 바로 상자를 봄)
//     팔 동작은 도착을 기다리지 않음 — 주행이 끝나도 계속 올라가고, approach가 같은 자세를 다시 보냄
//   goal.target "near" (상자 앞 다가설 자리, 11-13): 멈추거나(진척 없음) Nav2가 실패해도 목표에서 nav_near_m 안이면 성공 —
//     벽 옆 자리에서 옆으로 10 cm 남기고 MPPI가 못 줄여 실패하던 것. 마지막 몇 cm·방향은 approach가 상자를 보며 맞춤
//     방향이 nav_near_yaw보다 틀리면 성공 처리 안 함 (11-14: 상자에 붙어 못 돌고 등진 채 성공 → approach가 크게 돌다 실패)
//   target은 공백으로 여러 개 ("near look")
//   문 경유점 (doors, 11-15): 경로가 문을 대각선으로 질러 벽 끝에 코를 대고 collision_monitor가 세웠음 (실기 B3 6판 ·
//     시뮬 재현). 먼저 /compute_path_to_pose로 경로를 받아 지나는 문마다 들어가기 점(문 앞, 문 축 방향으로 제자리 회전) ·
//     나가기 점(문 너머, 일직선 유지)을 끼워 차례로 NavigateToPose. 목표가 문 바로 너머면 나가기 점은 뺌.
//     문은 경기장(arena) 좌표 — start_pose_arena로 map에 옮김 (door_use_correction이면 /arena/correction도 얹음 — 기본 끔)
#include <sstream>
#include "skills/arm_io.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/progress_watch.hpp"
#include "skills/skill_node.hpp"
#include "std_msgs/msg/string.hpp"

#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <set>
#include <vector>

namespace skills {

class NavigateSkill : public SkillNode {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using Pose = geometry_msgs::msg::PoseStamped;
  explicit NavigateSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_navigate", options), arm_(*this) {
    nav_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    path_ = rclcpp_action::create_client<ComputePath>(this, "/compute_path_to_pose");
    stall_s_ = declare_parameter<double>("nav_stall_s", 15.0);       // 0이면 감시 안 함
    stall_gain_m_ = declare_parameter<double>("nav_stall_gain_m", 0.10);
    look_m_ = declare_parameter<double>("nav_look_m", 0.6);
    near_m_ = declare_parameter<double>("nav_near_m", 0.15);
    near_yaw_ = declare_parameter<double>("nav_near_yaw", 0.8);  // [rad]
    look_q_ = declare_parameter<std::vector<double>>("pose_look", {0.0, -1.682497, 0.324631, 1.926843, 0.0});  // arm.yaml이 덮음
    doors_ = declare_parameter<std::vector<double>>("doors", std::vector<double>{});  // 문마다 7개 — skills.yaml 설명
    start_ = declare_parameter<std::vector<double>>("start_pose_arena", {0.734, 0.950, M_PI});
    door_goal_skip_m_ = declare_parameter<double>("door_goal_skip_m", 0.30);
    // 문에 /arena/correction을 얹을지 — 기본 끔 (11-15): 전역·local costmap은 /arena/walls(보정 안 한 실측 벽)를 봄. 문만 보정하면
    //   경로와 문 경유점이 다른 벽을 봐서, 돌아오는 길에 들어가기 점이 문 축에서 9 cm 어긋나 위쪽 벽 끝에 막혔음.
    //   보정은 이제 로봇 위치(map→odom)로 들어감 (mission_manager arena_corr_to_localization → cuvslam_slam) — 그러면 벽·문 모두 맞음
    door_use_corr_ = declare_parameter<bool>("door_use_correction", false);
    corr_sub_ = create_subscription<std_msgs::msg::String>(
      "/arena/correction", rclcpp::QoS(1).reliable().transient_local(), [this](std_msgs::msg::String::ConstSharedPtr m) {
        if (!door_use_corr_) return;
        for (const char * k : {"dx", "dy", "dth"}) {
          const auto i = m->data.find(std::string("\"") + k + "\":");
          if (i != std::string::npos) corr_[k] = std::strtod(m->data.c_str() + i + std::string(k).size() + 3, nullptr);
        }
      });
    serve("navigate", [this](const SkillGoal & g, Done done) {
      when_ready([this]() { return nav_->action_server_is_ready(); }, "nav_unavailable", done, [this, g, done]() {
        Pose goal = g.pose;
        if (goal.header.frame_id.empty()) goal.header.frame_id = "map";
        std::set<std::string> flags;
        std::istringstream in(g.target);
        for (std::string t; in >> t;) flags.insert(t);
        const uint64_t seq = ++nav_seq_;
        if (doors_.size() < 7 || !path_->action_server_is_ready()) {
          drive(std::make_shared<std::vector<Pose>>(1, goal), 0, flags, seq, done);
          return;
        }
        ComputePath::Goal pg;
        pg.goal = goal;
        pg.goal.header.stamp = now();
        pg.use_start = false;
        rclcpp_action::Client<ComputePath>::SendGoalOptions opt;
        opt.result_callback = [this, goal, flags, seq, done](const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult & r) {
          if (seq != nav_seq_) return;  // 그사이 멈춤
          auto legs = std::make_shared<std::vector<Pose>>();
          if (r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result) *legs = door_legs(r.result->path.poses, goal);
          legs->push_back(goal);  // 경로 실패면 문 없이 그대로 (Nav2가 다시 계획하며 실패를 알림)
          drive(legs, 0, flags, seq, done);
        };
        opt.goal_response_callback = [this, goal, flags, seq, done](const rclcpp_action::ClientGoalHandle<ComputePath>::SharedPtr & h) {
          if (!h && seq == nav_seq_) drive(std::make_shared<std::vector<Pose>>(1, goal), 0, flags, seq, done);
        };
        path_->async_send_goal(pg, opt);
      });
    });
  }

protected:
  void stop() override {
    ++nav_seq_;  // 응답 전인 goal은 goal_response_callback이 취소
    if (nav_goal_) nav_->async_cancel_goal(nav_goal_);
    nav_goal_.reset();
    arm_.stop();
  }

private:
  // 경로가 지나는 문 → 들어가기 · 나가기 점 (map). 문 한 줄 = cx, cy, nx, ny (문 축 단위벡터), half_w (문 폭 절반),
  //   d_neg, d_pos (축 −쪽 · +쪽 점까지 문 가운데에서 [m]) — 모두 arena
  std::vector<Pose> door_legs(const std::vector<Pose> & path, const Pose & goal) {
    std::vector<Pose> legs;
    if (path.size() < 2) return legs;
    // arena → map: 시작 자리 기준으로 돌리고 옮긴 뒤 /arena/correction (map' = R(dth)·map + d)
    const double th = -start_[2] + corr_["dth"], c = std::cos(th), s = std::sin(th);
    const double c0 = std::cos(-start_[2]), s0 = std::sin(-start_[2]), cd = std::cos(corr_["dth"]), sd = std::sin(corr_["dth"]);
    auto to_map = [&](double ax, double ay) {
      const double mx = c0 * (ax - start_[0]) - s0 * (ay - start_[1]), my = s0 * (ax - start_[0]) + c0 * (ay - start_[1]);
      return std::pair<double, double>{cd * mx - sd * my + corr_["dx"], sd * mx + cd * my + corr_["dy"]};
    };
    struct Hit { size_t k; size_t door; int sgn; };
    std::vector<Hit> hits;
    std::set<size_t> seen;
    for (size_t k = 0; k + 1 < path.size(); ++k) {
      const auto & a = path[k].pose.position, & b = path[k + 1].pose.position;
      for (size_t d = 0; d + 6 < doors_.size(); d += 7) {
        if (seen.count(d)) continue;
        const auto [cx, cy] = to_map(doors_[d], doors_[d + 1]);
        const double nx = c * doors_[d + 2] - s * doors_[d + 3], ny = s * doors_[d + 2] + c * doors_[d + 3];
        const double sa = (a.x - cx) * nx + (a.y - cy) * ny, sb = (b.x - cx) * nx + (b.y - cy) * ny;
        if ((sa < 0.0) == (sb < 0.0)) continue;
        const double t = sa / (sa - sb), qx = a.x + (b.x - a.x) * t, qy = a.y + (b.y - a.y) * t;
        if (std::abs(-(qx - cx) * ny + (qy - cy) * nx) > doors_[d + 4]) continue;
        seen.insert(d);
        hits.push_back({k, d, sb > sa ? 1 : -1});
      }
    }
    const auto & r0 = path.front().pose.position, & g = goal.pose.position;
    for (const auto & h : hits) {  // 경로 순서 (k 오름차순으로 찾았음)
      const size_t d = h.door;
      const auto [cx, cy] = to_map(doors_[d], doors_[d + 1]);
      const double nx = h.sgn * (c * doors_[d + 2] - s * doors_[d + 3]), ny = h.sgn * (s * doors_[d + 2] + c * doors_[d + 3]);
      const double back = h.sgn > 0 ? doors_[d + 5] : doors_[d + 6], fwd = h.sgn > 0 ? doors_[d + 6] : doors_[d + 5];
      auto point = [&](double along) {
        Pose p = goal;
        p.pose.position.x = cx + nx * along;
        p.pose.position.y = cy + ny * along;
        const double yaw = std::atan2(ny, nx);
        p.pose.orientation.x = p.pose.orientation.y = 0.0;
        p.pose.orientation.z = std::sin(yaw / 2);
        p.pose.orientation.w = std::cos(yaw / 2);
        return p;
      };
      if (std::hypot(r0.x - cx, r0.y - cy) > back + 0.10) legs.push_back(point(-back));  // 이미 문 앞이면 뺌
      if (std::hypot(g.x - cx, g.y - cy) > fwd + door_goal_skip_m_) legs.push_back(point(fwd));
      RCLCPP_INFO(get_logger(), "문 (%.2f, %.2f) 지남 — 경유 %zu개", cx, cy, legs.size());
    }
    return legs;
  }

  // legs[i]로 NavigateToPose — 성공하면 다음 점, 마지막 점에만 look · near (경유점은 그대로 도착해야 함)
  void drive(std::shared_ptr<std::vector<Pose>> legs, size_t i, std::set<std::string> flags, uint64_t seq, Done done) {
    if (seq != nav_seq_) return;
    const bool last = i + 1 == legs->size();
    NavigateToPose::Goal goal;
    goal.pose = (*legs)[i];
    goal.pose.header.stamp = now();
    stage(std::string(last ? "Nav2 (" : "Nav2 문 경유 (") + std::to_string(goal.pose.pose.position.x).substr(0, 5) + ", " +
          std::to_string(goal.pose.pose.position.y).substr(0, 5) + ")");
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
    auto finished = std::make_shared<bool>(false);  // 감시가 먼저 끝내면 취소 뒤 오는 결과는 무시
    auto watch = std::make_shared<ProgressWatch>(stall_s_, stall_gain_m_);
    auto look = std::make_shared<bool>(last && flags.count("look") > 0);  // true = 아직 팔을 안 올림
    const bool near = last && flags.count("near") > 0;
    auto left = std::make_shared<double>(1e9);  // 목표까지 직선 거리 (마지막 피드백)
    auto yaw_err = std::make_shared<double>(M_PI);  // 목표 방향과 차 [rad] (마지막 피드백)
    const auto gp = goal.pose.pose.position;
    const double gyaw = yaw_of(goal.pose.pose.orientation);
    // near: 끝날 때 목표 근처면 성공으로 바꿈 · 경유점 성공이면 다음 점
    auto finish = [this, near, left, yaw_err, done, legs, i, last, flags, seq](bool ok, const std::string & why) {
      if (!ok && near && *left <= near_m_ && *yaw_err <= near_yaw_) {
        RCLCPP_WARN(get_logger(), "%s — 목표 %.2f m · %.0f° 앞이라 성공 처리 (near, 나머지는 approach)", why.c_str(), *left,
                    *yaw_err * 180.0 / M_PI);
        ok = true;
      }
      if (ok && !last) { drive(legs, i + 1, flags, seq, done); return; }
      done(ok, ok ? "" : why);
    };
    if (stall_s_ > 0.0 || *look || near) {
      opt.feedback_callback = [this, finished, watch, look, left, yaw_err, gp, gyaw, finish](
          rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr h, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        if (*finished) return;
        *left = std::hypot(fb->current_pose.pose.position.x - gp.x, fb->current_pose.pose.position.y - gp.y);
        *yaw_err = std::abs(std::remainder(yaw_of(fb->current_pose.pose.orientation) - gyaw, 2 * M_PI));
        // 첫 피드백은 경로 전이라 0일 수 있음 → 0.01 m 넘을 때만
        if (*look && fb->distance_remaining > 0.01 && fb->distance_remaining < look_m_) {
          *look = false;
          if (arm_.moveit_ready())
            arm_.move_joints(look_q_, [this](bool ok) { if (!ok) RCLCPP_WARN(get_logger(), "주행 중 look 자세 실패"); });
          else RCLCPP_WARN(get_logger(), "MoveIt 없음 — 주행 중 look 자세 건너뜀");
        }
        if (stall_s_ <= 0.0 || !watch->update(now().seconds(), fb->distance_remaining)) return;
        *finished = true;
        RCLCPP_WARN(get_logger(), "진척 없음 — 남은 %.2f m가 %.0f s 동안 %.2f m도 안 줄음 (회복 %d회) → 취소",
                    fb->distance_remaining, stall_s_, stall_gain_m_, fb->number_of_recoveries);
        nav_->async_cancel_goal(h);
        finish(false, "nav_no_progress");
      };
    }
    // 이 goal만 취소하려고 손잡이를 보관 — stop()이 cancel_all을 보내면 바로 뒤 새 계획의 Nav2 goal까지 취소돼
    //   새 navigate가 시작하자마자 nav_failed였음 (실기 11-15 approach_loop 재계획)
    opt.goal_response_callback = [this, seq, finished, finish](
        const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & h) {
      if (seq != nav_seq_) {  // 응답 전에 멈춤 — 늦게 받아진 goal도 취소
        if (h) nav_->async_cancel_goal(h);
        return;
      }
      if (h) { nav_goal_ = h; return; }
      if (*finished) return;  // 거절되면 결과가 오지 않음 — 여기서 끝내야 skill이 안 멈춤
      *finished = true;
      finish(false, "nav_rejected");
    };
    opt.result_callback = [this, seq, finished, finish](
        const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
      if (seq == nav_seq_) nav_goal_.reset();
      if (*finished || seq != nav_seq_) return;
      *finished = true;
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
      finish(ok, ok ? "" : "nav_failed");
    };
    nav_->async_send_goal(goal, opt);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
  rclcpp_action::Client<ComputePath>::SharedPtr path_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr corr_sub_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_goal_;  // 지금 보낸 Nav2 goal (stop이 이것만 취소)
  uint64_t nav_seq_ = 0;  // 보낸 goal 번호 — stop하면 올라가 옛 응답·결과를 가려냄
  static double yaw_of(const geometry_msgs::msg::Quaternion & q) {
    return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }
  double stall_s_ = 25.0, stall_gain_m_ = 0.10, look_m_ = 0.6, near_m_ = 0.15, near_yaw_ = 0.8, door_goal_skip_m_ = 0.30;
  std::vector<double> doors_, start_;
  std::map<std::string, double> corr_{{"dx", 0.0}, {"dy", 0.0}, {"dth", 0.0}};
  bool door_use_corr_ = false;
  ArmIO arm_;
  std::vector<double> look_q_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::NavigateSkill)
