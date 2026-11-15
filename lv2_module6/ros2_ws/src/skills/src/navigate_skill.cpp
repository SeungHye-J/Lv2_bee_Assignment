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
//   꼭 거칠 길목 (gates, 11-15 사용자 지도): A → B 화살표마다 — 경로가 A→B 가운데 수직선을 그 방향으로 건너면 A · B를 차례로
//     경유 (both면 거꾸로도 B · A). 문과 겹치면(문 가운데가 A–B 선분에서 0.4 m 안) 그 문은 뺌. A·B는 사용자가 찍은 점 그대로
//   가르친 경로 (routes_dir, 11-15 teach & repeat): 사람이 수동으로 몬 경로(tools/field/route_record.py)의 시작이 로봇 근처 ·
//     끝이 목표 근처면(거꾸로도) 문 경유점 대신 그 경로의 꺾이는 점들을 경유점으로 (skills/route_plan.hpp)
//   움직임 정답표 (taught_routes, 11-15): goal.target에 "leg:<구간>"(mission_manager가 붙임 — 예 S4_S5)이 있고
//     routes_dir에 *_<구간>.motion.csv가 있으면 Nav2 경로 계획 대신 표대로 Spin(회전) · DriveOnHeading(직진)을 차례로.
//     동작마다 odom 자세를 표의 기대 자세와 비교해 route_dev_m · route_dev_deg 넘게 엇나가거나 동작이 실패하면
//     남은 길은 예전처럼 Nav2로 (실패 아님). 속도 = 같은 구간 .cmd.csv의 사람 직진 속도 중앙값
//   경유점 지도 (via_points, 11-15 사용자 지정): 사람이 찍은 점 + 화살표 — 보이는 점끼리 최단으로 이어 경유점 (skills/via_graph.hpp).
//     가르친 경로가 없을 때 문 경유점 대신 씀 (로봇·목표가 어느 점과도 못 이어지면 문 경유점으로)
//     via_through(기본 true, 11-16): 경유점 + 목표를 NavigateThroughPoses 한 번으로 — 경유점마다 서서 돌지 않고 지나감
//       (Nav2 BT RemovePassedGoals radius 0.40 m 안이면 지난 것). 마지막 목표의 near · look · 진척 감시 · 끼임 풀기는 같음.
//       서버가 없으면 예전처럼 경유점마다 NavigateToPose. 문 경유점 · 가르친 경로는 정확히 서야 해서 그대로 NavigateToPose
//   끼임 풀기 (11-15): 진척 없음이면 바로 실패하지 않고 nav_unstuck_tries번까지 — 곧게 후진(nav_unstuck_back_m) → 그 자리에서 다시 받은
//     경로의 앞(0.4 m) 쪽으로 제자리 회전 → 같은 점으로 다시 출발. 시뮬 B4→B5: 문 모서리에 코를 대고 Nav2 복구(후진 0.10 m)로도
//     같은 대각선을 다시 타서 멈춤
#include <sstream>
#include "skills/arm_io.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/progress_watch.hpp"
#include "skills/route_plan.hpp"
#include "skills/via_graph.hpp"
#include "skills/skill_node.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <vector>

namespace skills {

class NavigateSkill : public SkillNode {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThrough = nav2_msgs::action::NavigateThroughPoses;
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using BackUp = nav2_msgs::action::BackUp;
  using Spin = nav2_msgs::action::Spin;
  using Pose = geometry_msgs::msg::PoseStamped;
  using Drive = nav2_msgs::action::DriveOnHeading;
  using Follow = nav2_msgs::action::FollowPath;
  explicit NavigateSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_navigate", options), arm_(*this) {
    nav_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    through_ = rclcpp_action::create_client<NavigateThrough>(this, "/navigate_through_poses");
    path_ = rclcpp_action::create_client<ComputePath>(this, "/compute_path_to_pose");
    spin_ = rclcpp_action::create_client<Spin>(this, "/spin");
    drive_ = rclcpp_action::create_client<Drive>(this, "/drive_on_heading");
    follow_ = rclcpp_action::create_client<Follow>(this, "/follow_path");
    backup_ = rclcpp_action::create_client<BackUp>(this, "/backup");
    unstuck_tries_ = declare_parameter<int>("nav_unstuck_tries", 2);     // 0이면 끼임 풀기 안 함
    unstuck_back_m_ = declare_parameter<double>("nav_unstuck_back_m", 0.15);
    stall_s_ = declare_parameter<double>("nav_stall_s", 15.0);       // 0이면 감시 안 함
    stall_gain_m_ = declare_parameter<double>("nav_stall_gain_m", 0.10);
    look_m_ = declare_parameter<double>("nav_look_m", 0.6);
    near_m_ = declare_parameter<double>("nav_near_m", 0.15);
    near_yaw_ = declare_parameter<double>("nav_near_yaw", 0.8);  // [rad]
    via_through_ = declare_parameter<bool>("via_through", true);  // 경유점 지도 길을 NavigateThroughPoses 한 번으로 (머리말)
    look_q_ = declare_parameter<std::vector<double>>("pose_look", {0.0, -1.682497, 0.324631, 1.926843, 0.0});  // arm.yaml이 덮음
    doors_ = declare_parameter<std::vector<double>>("doors", std::vector<double>{});  // 문마다 7개 — skills.yaml 설명
    gates_ = declare_parameter<std::vector<double>>("gates", std::vector<double>{});  // 길목마다 6개 — skills.yaml 설명
    no_path_ = declare_parameter<std::vector<double>>("no_path_zones", std::vector<double>{});  // 사각형마다 4개 — skills.yaml 설명
    start_ = declare_parameter<std::vector<double>>("start_pose_arena", {0.734, 0.950, M_PI});
    door_goal_skip_m_ = declare_parameter<double>("door_goal_skip_m", 0.30);
    // 문에 /arena/correction을 얹을지 — 기본 끔 (11-15): 전역·local costmap은 /arena/walls(보정 안 한 실측 벽)를 봄. 문만 보정하면
    //   경로와 문 경유점이 다른 벽을 봐서, 돌아오는 길에 들어가기 점이 문 축에서 9 cm 어긋나 위쪽 벽 끝에 막혔음.
    //   보정은 이제 로봇 위치(map→odom)로 들어감 (mission_manager arena_corr_to_localization → cuvslam_slam) — 그러면 벽·문 모두 맞음
    door_use_corr_ = declare_parameter<bool>("door_use_correction", false);
    routes_dir_ = declare_parameter<std::string>("routes_dir", "~/.ros/routes");  // 가르친 경로 *.txt ('' = 안 씀)
    if (routes_dir_.rfind("~/", 0) == 0 && std::getenv("HOME")) routes_dir_ = std::string(std::getenv("HOME")) + routes_dir_.substr(1);
    route_start_m_ = declare_parameter<double>("route_start_m", 0.5);   // 경로 시작이 로봇에서 이 안이어야
    route_goal_m_ = declare_parameter<double>("route_goal_m", 0.4);     // 경로 끝이 목표에서 이 안이어야
    route_tol_m_ = declare_parameter<double>("route_simplify_m", 0.10); // 꺾이는 점만 남길 때 허용 오차
    taught_ = declare_parameter<bool>("taught_routes", false);          // 움직임 정답표 재생 (launch taught_routes:=true)
    dev_m_ = declare_parameter<double>("route_dev_m", 0.30);            // 표 기대 자세에서 이만큼 벗어나면 Nav2로
    dev_rad_ = declare_parameter<double>("route_dev_deg", 15.0) * M_PI / 180.0;
    route_speed_ = declare_parameter<double>("route_speed", 0.15);      // .cmd.csv가 없을 때 직진 속도 [m/s]
    fix_rad_ = declare_parameter<double>("route_heading_fix_deg", 3.0) * M_PI / 180.0;  // 직진 전 이보다 틀리면 먼저 돎
    // 회전 뒤 이만큼 기다렸다 방향을 잼 — Spin은 목표 각도에 닿는 순간 끝을 알리고 감속하며 더 돎 (시뮬 정답: −86° → −98.7°)
    settle_s_ = declare_parameter<double>("route_settle_s", 0.8);
    // 엇나가면: false(기본) = 멈추고 실패 (표가 맞는지 시험) · true = 남은 길은 Nav2
    fallback_nav2_ = declare_parameter<bool>("route_fallback_nav2", false);
    // follow(기본) = 표를 map 경로로 바꿔 FollowPath — 컨트롤러가 local costmap(벽·장애물)을 보며 따라감 (사람이 몬 길 + 주변 인지)
    //   primitives = Spin · DriveOnHeading으로 거리·각도만 (눈 감고 셈 — 시뮬: odom 회전이 12% 많게 잡혀 10° 틀어져 벽으로)
    route_mode_ = declare_parameter<std::string>("route_mode", "follow");
    anchor_ = declare_parameter<bool>("route_anchor_end", true);
    // 표대로면 끝이 목표에서 이보다 멀면 그 표는 안 믿고 Nav2 (11-15 시뮬: S5_S3 1.80 m — 녹화 때 시작 방향이 틀린 표 · 억지로 맞추면 경로가 휘어 막힘)
    max_end_err_ = declare_parameter<double>("route_max_end_err_m", 0.6);
    // Nav2가 '출발 칸이 막힘'(205)이면 이만큼 뒤로 빼고 한 번 다시 (상자·벽에 붙은 채 실패한 뒤 다음 이동이 전부 205로 끝났음)
    unstick_m_ = declare_parameter<double>("nav_unstick_backup_m", 0.15);  // follow 경로 끝을 Nav2 목표(벽 여유를 둔 다가설 자리)에 맞춤
    // 경유점 지도 — 모두 arena: 점 [x, y, …] · 화살표 [a, b, …] (점 번호) · 벽 [x0, y0, x1, y1, …] · 상자 [x, y, 반 변, …]
    {
      const auto pts = declare_parameter<std::vector<double>>("via_points", std::vector<double>{});
      const auto arr = declare_parameter<std::vector<int64_t>>("via_arrows", std::vector<int64_t>{});
      const auto walls = declare_parameter<std::vector<double>>("via_walls", std::vector<double>{});
      const auto boxes = declare_parameter<std::vector<double>>("via_boxes", std::vector<double>{});
      via_.clearance = declare_parameter<double>("via_clearance", 0.18);
      const double c0 = std::cos(-start_[2]), s0 = std::sin(-start_[2]);
      auto to_map = [&](double ax, double ay) {  // arena → map (문과 같은 식)
        ax -= start_[0]; ay -= start_[1];
        return ViaGraph::P{c0 * ax - s0 * ay, s0 * ax + c0 * ay};
      };
      for (size_t i = 0; i + 1 < pts.size(); i += 2) via_.nodes.push_back(to_map(pts[i], pts[i + 1]));
      for (size_t i = 0; i + 1 < arr.size(); i += 2) via_.arrows.push_back({static_cast<int>(arr[i]), static_cast<int>(arr[i + 1])});
      for (size_t i = 0; i + 3 < walls.size(); i += 4) via_.obstacles.push_back({to_map(walls[i], walls[i + 1]), to_map(walls[i + 2], walls[i + 3])});
      for (size_t i = 0; i + 2 < boxes.size(); i += 3) {
        const auto [mx, my] = to_map(boxes[i], boxes[i + 1]);
        via_.add_box(mx, my, boxes[i + 2]);  // 반 변 축 정렬 — start yaw가 π라 map에서도 축 정렬
      }
    }
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
        std::string leg;
        for (const auto & f : flags) if (f.rfind("leg:", 0) == 0) leg = f.substr(4);
        if (taught_ && !leg.empty() && start_replay(leg, goal, flags, seq, done)) return;
        nav2(goal, flags, seq, done);
      });
    });
  }

protected:
  void stop() override {
    ++nav_seq_;  // 응답 전인 goal은 goal_response_callback이 취소
    if (nav_goal_) nav_->async_cancel_goal(nav_goal_);
    nav_goal_.reset();
    if (through_goal_) through_->async_cancel_goal(through_goal_);
    through_goal_.reset();
    if (spin_goal_) spin_->async_cancel_goal(spin_goal_);
    spin_goal_.reset();
    if (drive_goal_) drive_->async_cancel_goal(drive_goal_);
    drive_goal_.reset();
    if (follow_goal_) follow_->async_cancel_goal(follow_goal_);
    follow_goal_.reset();
    arm_.stop();
  }

private:
  // 예전 길: 플래너 경로 → 가르친 경로(map) 또는 문 경유점 → NavigateToPose
  void nav2(const Pose & goal, const std::set<std::string> & flags, uint64_t seq, Done done) {
    if ((doors_.size() < 7 && gates_.size() < 6 && via_.nodes.empty() && no_path_.size() < 4) || !path_->action_server_is_ready()) {
      drive(std::make_shared<std::vector<Pose>>(1, goal), 0, flags, seq, done);
      return;
    }
    ComputePath::Goal pg;
    pg.goal = goal;
    pg.goal.header.stamp = builtin_interfaces::msg::Time();  // 0 = 가장 최근 TF (아래 drive 참고)
    pg.use_start = false;
    rclcpp_action::Client<ComputePath>::SendGoalOptions opt;
    opt.result_callback = [this, goal, flags, seq, done](const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult & r) {
      if (seq != nav_seq_) return;  // 그사이 멈춤
      auto legs = std::make_shared<std::vector<Pose>>();
      bool via = false;  // 경유점 지도 길 (via_through면 NavigateThroughPoses 한 번)
      if (r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result && !r.result->path.poses.empty()) {
        // 경유점 지도 먼저 (11-16) → 이어진 길이 없으면 가르친 경로 → 그것도 없으면 막은 구역 검사 → 문 경유점
        //   경유 0개(로봇과 목표가 바로 이어짐)도 경유점 지도의 답 — 가르친 경로로 넘어가지 않음
        const auto & rp = r.result->path.poses.front().pose.position;
        const auto v = via_.nodes.empty() ? std::nullopt
                                          : via_.route({rp.x, rp.y}, {goal.pose.position.x, goal.pose.position.y});
        if (v) {
          for (size_t k = 0; k < v->size(); ++k) {
            const auto nxt = k + 1 < v->size() ? (*v)[k + 1] : ViaGraph::P{goal.pose.position.x, goal.pose.position.y};
            const double yaw = std::atan2(nxt.second - (*v)[k].second, nxt.first - (*v)[k].first);  // 다음 점을 봄
            Pose p = goal;
            p.pose.position.x = (*v)[k].first;
            p.pose.position.y = (*v)[k].second;
            p.pose.orientation.x = p.pose.orientation.y = 0.0;
            p.pose.orientation.z = std::sin(yaw / 2);
            p.pose.orientation.w = std::cos(yaw / 2);
            legs->push_back(p);
          }
          RCLCPP_INFO(get_logger(), "경유점 지도 — 경유 %zu개", legs->size());
          via = !legs->empty();
        } else {
          if (!via_.nodes.empty()) RCLCPP_WARN(get_logger(), "경유점 지도에 이어진 길 없음 — 가르친 경로 · 문 경유점으로");
          *legs = route_legs(r.result->path.poses.front(), goal);  // 가르친 경로가 있으면 그것 (문도 이미 사람이 지남)
          if (legs->empty()) {
            if (const int z = blocked_zone(r.result->path.poses); z >= 0) {  // 그 구역엔 경유점이 없음 — 돌아갈 길도 없다고 봄
              RCLCPP_WARN(get_logger(), "경로가 막은 구역 %d(no_path_zones)을 지남 — 안 감", z);
              done(false, "nav_no_path_zone");
              return;
            }
            *legs = door_legs(r.result->path.poses, goal);
          }
        }
      }
      legs->push_back(goal);  // 경로 실패면 문 없이 그대로 (Nav2가 다시 계획하며 실패를 알림)
      if (via && via_through_ && through_->action_server_is_ready()) drive_through(legs, flags, seq, done);
      else drive(legs, 0, flags, seq, done);
    };
    opt.goal_response_callback = [this, goal, flags, seq, done](const rclcpp_action::ClientGoalHandle<ComputePath>::SharedPtr & h) {
      if (!h && seq == nav_seq_) drive(std::make_shared<std::vector<Pose>>(1, goal), 0, flags, seq, done);
    };
    path_->async_send_goal(pg, opt);
  }

  // Spin (odom 기준 상대 각도) — 결과를 cb(ok, why)로
  void send_spin(double yaw, uint64_t seq, std::function<void(bool, const std::string &)> cb) {
    Spin::Goal g;
    g.target_yaw = static_cast<float>(yaw);
    g.time_allowance.sec = static_cast<int32_t>(std::abs(yaw) / 0.3 + 10.0);
    g.disable_collision_checks = true;  // 사람이 몬 길 — 벽 옆 회전도 그대로 (collision_monitor는 계속 켜짐)
    rclcpp_action::Client<Spin>::SendGoalOptions opt;
    opt.goal_response_callback = [this, seq, cb](const rclcpp_action::ClientGoalHandle<Spin>::SharedPtr & h) {
      if (seq != nav_seq_) { if (h) spin_->async_cancel_goal(h); return; }
      if (h) spin_goal_ = h; else cb(false, "spin_rejected");
    };
    opt.result_callback = [this, cb](const rclcpp_action::ClientGoalHandle<Spin>::WrappedResult & r) {
      spin_goal_.reset();
      cb(r.code == rclcpp_action::ResultCode::SUCCEEDED, "spin_failed");
    };
    spin_->async_send_goal(g, opt);
  }

  bool odom_pose(Pose2d & p, const std::string & frame = "odom") {
    try {
      const auto t = arm_.tf().lookupTransform(frame, "base_footprint", tf2::TimePointZero).transform;
      p = {t.translation.x, t.translation.y, yaw_of(t.rotation)};
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  // 구간 표 찾기 → 재생 시작. 표·액션 서버·odom이 없으면 false (부르는 쪽이 Nav2로)
  bool start_replay(const std::string & leg, const Pose & goal, const std::set<std::string> & flags, uint64_t seq, Done done) {
    std::error_code ec;
    std::string best;
    std::filesystem::file_time_type best_t{};
    const std::string tail = "_" + leg + ".motion.csv";
    if (!routes_dir_.empty() && std::filesystem::is_directory(routes_dir_, ec))
      for (const auto & e : std::filesystem::directory_iterator(routes_dir_, ec)) {
        const std::string n = e.path().filename().string();
        if (n.size() < tail.size() || n.compare(n.size() - tail.size(), tail.size(), tail) != 0) continue;
        const auto t = std::filesystem::last_write_time(e.path(), ec);
        if (best.empty() || t > best_t) { best = e.path().string(); best_t = t; }  // 다시 녹화한 최신 것
      }
    if (best.empty()) { RCLCPP_INFO(get_logger(), "구간 %s 표 없음 — Nav2", leg.c_str()); return false; }
    auto moves = std::make_shared<std::vector<Move>>(load_motion(best));
    if (route_mode_ == "follow" && !moves->empty()) return start_follow(leg, best, *moves, goal, flags, seq, done);
    Pose2d p;
    if (moves->empty() || !spin_->action_server_is_ready() || !drive_->action_server_is_ready() || !odom_pose(p)) {
      RCLCPP_WARN(get_logger(), "구간 %s 표 %s — 재생 못 함 (빈 표 · /spin·/drive_on_heading 없음 · odom TF 없음) → Nav2",
                  leg.c_str(), best.c_str());
      return false;
    }
    const double speed = std::clamp(median_speed(best.substr(0, best.size() - 11) + ".cmd.csv", route_speed_), 0.05, 0.22);
    RCLCPP_INFO(get_logger(), "구간 %s 표대로 (%zu동작 · 직진 %.2f m/s) — %s", leg.c_str(), moves->size(), speed, best.c_str());
    replay(leg, moves, 0, p, speed, goal, flags, seq, done);
    return true;
  }

  // follow: 표 → map 경로 (지금 로봇 자세에서) → FollowPath. 실패하면 멈춤 (route_fallback_nav2면 Nav2)
  bool start_follow(const std::string & leg, const std::string & file, const std::vector<Move> & moves, const Pose & goal,
                    const std::set<std::string> & flags, uint64_t seq, Done done) {
    Pose2d p;
    if (!follow_->action_server_is_ready() || !odom_pose(p, "map")) {
      RCLCPP_WARN(get_logger(), "구간 %s — /follow_path 없음 또는 map TF 없음 → Nav2", leg.c_str());
      return false;
    }
    Follow::Goal g;
    // 이름을 꼭 줘야 함 — goal_checker가 둘이라 빈 이름이면 "Failed to find goal checker name" (BT와 같은 이름: nav2_params.yaml)
    g.controller_id = "FollowPath";
    g.goal_checker_id = "general_goal_checker";
    g.progress_checker_id = "progress_checker";
    g.path.header.frame_id = "map";
    g.path.header.stamp = now();
    auto pts = path_points(p, moves);
    const double raw_end = std::hypot(pts.back().x - goal.pose.position.x, pts.back().y - goal.pose.position.y);
    if (raw_end > max_end_err_) {
      RCLCPP_WARN(get_logger(), "구간 %s 표대로면 끝이 목표와 %.2f m (> %.2f) — 표를 안 믿고 Nav2 (다시 녹화 필요)", leg.c_str(), raw_end,
                  max_end_err_);
      return false;
    }
    if (anchor_) anchor_end(pts, {goal.pose.position.x, goal.pose.position.y, yaw_of(goal.pose.orientation)});
    for (const auto & q : pts) {
      Pose ps;
      ps.header = g.path.header;
      ps.pose.position.x = q.x;
      ps.pose.position.y = q.y;
      ps.pose.orientation.z = std::sin(q.yaw / 2);
      ps.pose.orientation.w = std::cos(q.yaw / 2);
      g.path.poses.push_back(ps);
    }
    const auto & e = g.path.poses.back().pose.position;
    RCLCPP_INFO(get_logger(), "구간 %s 표 → 경로 %zu점 (표대로면 끝이 목표와 %.2f m%s · 끝 map %.2f, %.2f) FollowPath — %s", leg.c_str(),
                g.path.poses.size(), raw_end, anchor_ ? " → 목표에 맞춤" : "", e.x, e.y, file.c_str());
    stage("표 " + leg + " 경로 따라가기");
    send_follow(std::make_shared<Follow::Goal>(g), leg, goal, flags, seq, done);
    return true;
  }

  // FollowPath 보내기 — 걸리면(실패) 이 seq에서 한 번 뒤로 빼고, 지금 자리에서 가장 가까운 점부터 남은 경로로 다시 (11-16 시뮬: 문 모서리에 걸려 실패)
  void send_follow(std::shared_ptr<Follow::Goal> g, const std::string & leg, const Pose & goal, const std::set<std::string> & flags,
                   uint64_t seq, Done done) {
    rclcpp_action::Client<Follow>::SendGoalOptions opt;
    const auto finish = [this, g, leg, goal, flags, seq, done](bool ok, const std::string & why) {
      if (seq != nav_seq_) return;
      if (ok) { RCLCPP_INFO(get_logger(), "구간 %s 경로 끝 — 도착", leg.c_str()); done(true, ""); return; }
      if (unstuck_seq_ != seq && unstick(seq, [this, g, leg, goal, flags, seq, done]() {
            Pose2d p;
            if (odom_pose(p, "map") && g->path.poses.size() > 2) {
              size_t k = 0;
              double best = 1e9;
              for (size_t j = 0; j < g->path.poses.size(); ++j) {
                const auto & q = g->path.poses[j].pose.position;
                const double d = std::hypot(q.x - p.x, q.y - p.y);
                if (d < best) { best = d; k = j; }
              }
              g->path.poses.erase(g->path.poses.begin(), g->path.poses.begin() + static_cast<long>(std::min(k, g->path.poses.size() - 2)));
              g->path.header.stamp = now();
              RCLCPP_WARN(get_logger(), "구간 %s 걸림 뒤 남은 경로 %zu점으로 다시", leg.c_str(), g->path.poses.size());
            }
            send_follow(g, leg, goal, flags, seq, done);
          })) return;
      if (!fallback_nav2_) {
        RCLCPP_WARN(get_logger(), "구간 %s 경로 따라가기 실패 (%s) — 멈춤", leg.c_str(), why.c_str());
        done(false, "route_follow_failed");
        return;
      }
      RCLCPP_WARN(get_logger(), "구간 %s 경로 따라가기 실패 (%s) — Nav2", leg.c_str(), why.c_str());
      nav2(goal, flags, seq, done);
    };
    opt.goal_response_callback = [this, seq, finish](const rclcpp_action::ClientGoalHandle<Follow>::SharedPtr & h) {
      if (seq != nav_seq_) { if (h) follow_->async_cancel_goal(h); return; }
      if (h) follow_goal_ = h; else finish(false, "follow_rejected");
    };
    opt.result_callback = [this, finish](const rclcpp_action::ClientGoalHandle<Follow>::WrappedResult & r) {
      follow_goal_.reset();
      finish(r.code == rclcpp_action::ResultCode::SUCCEEDED,
             r.result ? fmt("error %u %s", static_cast<unsigned>(r.result->error_code), r.result->error_msg.c_str()) : "aborted");
    };
    follow_->async_send_goal(*g, opt);
  }

  // 표의 i번째 동작 — 끝나면 odom을 기대 자세와 비교. 엇나가거나 실패하면 남은 길은 Nav2로
  //   회전은 '표의 기대 방향'까지 (지난 회전이 더 돌았거나 직진 중 휜 것까지 메움 — 11-15 실기: Spin −86°가 −99°로 13° 더 돌아
  //   2.2 m 직진 뒤 0.56 m 엇나감). 직진 전에 방향이 route_heading_fix_deg 넘게 틀리면 먼저 그만큼 돎 (한 번)
  void replay(const std::string & leg, std::shared_ptr<std::vector<Move>> moves, size_t i, Pose2d expect, double speed,
              const Pose & goal, const std::set<std::string> & flags, uint64_t seq, Done done, bool fixed = false) {
    if (seq != nav_seq_) return;
    if (i >= moves->size()) {
      RCLCPP_INFO(get_logger(), "구간 %s 표 끝 — 도착", leg.c_str());
      done(true, "");
      return;
    }
    const Move m = (*moves)[i];
    const Pose2d next = after(expect, m);
    const auto fallback = [this, leg, goal, flags, seq, done](const std::string & why) {
      if (seq != nav_seq_) return;
      if (!fallback_nav2_) {
        RCLCPP_WARN(get_logger(), "구간 %s 표 재생 그만 (%s) — 멈춤 (route_fallback_nav2: false)", leg.c_str(), why.c_str());
        done(false, "route_deviation");
        return;
      }
      RCLCPP_WARN(get_logger(), "구간 %s 표 재생 그만 (%s) — 남은 길은 Nav2", leg.c_str(), why.c_str());
      nav2(goal, flags, seq, done);
    };
    Pose2d cur;
    if (!odom_pose(cur)) { fallback("odom_tf"); return; }
    const double head_err = std::remainder(expect.yaw - cur.yaw, 2 * M_PI);
    if (!m.turn && !fixed && std::abs(head_err) > fix_rad_) {  // 직진 전 방향 바로잡기
      stage(fmt("표 %s %zu/%zu 방향 바로잡기 %+.0f°", leg.c_str(), i + 1, moves->size(), head_err * 180.0 / M_PI));
      send_spin(head_err, seq, [this, leg, moves, i, expect, speed, goal, flags, seq, done, fallback](bool ok, const std::string & why) {
        if (seq != nav_seq_) return;
        if (!ok) { fallback(why); return; }
        replay(leg, moves, i, expect, speed, goal, flags, seq, done, true);
      });
      return;
    }
    const auto check = [this, leg, moves, i, m, next, speed, goal, flags, seq, done, fallback](bool ok, const std::string & why) {
      if (seq != nav_seq_) return;
      if (!ok) { fallback(why); return; }
      Pose2d now_p;
      if (!odom_pose(now_p)) { fallback("odom_tf"); return; }
      const auto [dm, dr] = deviation(next, now_p);
      // 회전 뒤 방향 차는 다음 직진 전에 바로잡으니 위치만 봄 · 직진 뒤엔 둘 다
      if (dm > dev_m_ || (!m.turn && dr > dev_rad_)) {
        fallback(fmt("엇나감 %.2f m · %.0f°", dm, dr * 180.0 / M_PI));
        return;
      }
      if (!m.turn) { replay(leg, moves, i + 1, next, speed, goal, flags, seq, done); return; }
      // 회전 뒤: 멈출 때까지 기다렸다가 다음 동작 (다음 직진 전 방향 바로잡기가 남은 회전까지 봄)
      auto t = std::make_shared<rclcpp::TimerBase::SharedPtr>();
      *t = create_wall_timer(std::chrono::duration<double>(settle_s_), [this, t, leg, moves, i, next, speed, goal, flags, seq, done]() {
        (*t)->cancel();
        replay(leg, moves, i + 1, next, speed, goal, flags, seq, done);
      });
    };
    stage(fmt("표 %s %zu/%zu %s", leg.c_str(), i + 1, moves->size(),
              (m.turn ? fmt("회전 %+.0f°", m.value * 180.0 / M_PI) : fmt("직진 %.2f m", m.value)).c_str()));
    if (m.turn) {
      send_spin(std::remainder(next.yaw - cur.yaw, 2 * M_PI), seq, check);  // 표의 기대 방향까지
    } else {
      Drive::Goal g;
      g.target.x = m.value;
      g.speed = static_cast<float>(speed);
      g.time_allowance.sec = static_cast<int32_t>(m.value / speed * 2.0 + 10.0);
      g.disable_collision_checks = true;
      rclcpp_action::Client<Drive>::SendGoalOptions opt;
      opt.goal_response_callback = [this, seq, check](const rclcpp_action::ClientGoalHandle<Drive>::SharedPtr & h) {
        if (seq != nav_seq_) { if (h) drive_->async_cancel_goal(h); return; }
        if (h) drive_goal_ = h; else check(false, "drive_rejected");
      };
      opt.result_callback = [this, check](const rclcpp_action::ClientGoalHandle<Drive>::WrappedResult & r) {
        drive_goal_.reset();
        check(r.code == rclcpp_action::ResultCode::SUCCEEDED, "drive_failed");
      };
      drive_->async_send_goal(g, opt);
    }
  }

  // map → arena (door_legs의 to_map 거꾸로: a = R(yaw0)·R(−dth)·(m − d) + start)
  std::pair<double, double> to_arena(double x, double y) {
    const double ux = x - corr_["dx"], uy = y - corr_["dy"], cd = std::cos(corr_["dth"]), sd = std::sin(corr_["dth"]);
    const double mx = cd * ux + sd * uy, my = -sd * ux + cd * uy, c0 = std::cos(start_[2]), s0 = std::sin(start_[2]);
    return {c0 * mx - s0 * my + start_[0], s0 * mx + c0 * my + start_[1]};
  }

  // 경로 점이 막은 구역(no_path_zones, arena 사각형) 안이면 그 구역 번호 (0부터), 아니면 −1
  int blocked_zone(const std::vector<Pose> & path) {
    for (const auto & p : path) {
      const auto [ax, ay] = to_arena(p.pose.position.x, p.pose.position.y);
      for (size_t i = 0; i + 3 < no_path_.size(); i += 4)
        if (ax >= std::min(no_path_[i], no_path_[i + 2]) && ax <= std::max(no_path_[i], no_path_[i + 2]) &&
            ay >= std::min(no_path_[i + 1], no_path_[i + 3]) && ay <= std::max(no_path_[i + 1], no_path_[i + 3]))
          return static_cast<int>(i / 4);
    }
    return -1;
  }

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
    // 문 · 길목을 한 모양으로 (arena): 가운데 · 축 · 폭 절반 · 축 −쪽/+쪽 점 거리. 길목은 A–B 가운데 · 축 A→B · 점 = A, B
    struct Cross { double cx, cy, nx, ny, half_w, d_neg, d_pos; bool gate, one_way; double ax, ay, bx, by; };
    std::vector<Cross> xs;
    for (size_t d = 0; d + 6 < doors_.size(); d += 7)
      xs.push_back({doors_[d], doors_[d + 1], doors_[d + 2], doors_[d + 3], doors_[d + 4], doors_[d + 5], doors_[d + 6], false, false,
                    0, 0, 0, 0});
    for (size_t i = 0; i + 5 < gates_.size(); i += 6) {
      const double ax = gates_[i], ay = gates_[i + 1], bx = gates_[i + 2], by = gates_[i + 3], len = std::hypot(bx - ax, by - ay);
      if (len < 1e-3) continue;
      xs.push_back({(ax + bx) / 2, (ay + by) / 2, (bx - ax) / len, (by - ay) / len, gates_[i + 4], len / 2, len / 2, true,
                    gates_[i + 5] < 0.5, ax, ay, bx, by});
    }
    struct Hit { size_t k; size_t x; int sgn; };
    std::vector<Hit> hits;
    std::set<size_t> seen;
    for (size_t k = 0; k + 1 < path.size(); ++k) {
      const auto & a = path[k].pose.position, & b = path[k + 1].pose.position;
      for (size_t x = 0; x < xs.size(); ++x) {
        if (seen.count(x)) continue;
        const auto & X = xs[x];
        const auto [cx, cy] = to_map(X.cx, X.cy);
        const double nx = c * X.nx - s * X.ny, ny = s * X.nx + c * X.ny;
        const double sa = (a.x - cx) * nx + (a.y - cy) * ny, sb = (b.x - cx) * nx + (b.y - cy) * ny;
        if ((sa < 0.0) == (sb < 0.0)) continue;
        const double t = sa / (sa - sb), qx = a.x + (b.x - a.x) * t, qy = a.y + (b.y - a.y) * t;
        if (std::abs(-(qx - cx) * ny + (qy - cy) * nx) > X.half_w) continue;
        const int sgn = sb > sa ? 1 : -1;
        if (X.one_way && sgn < 0) continue;  // 화살표 반대로 지나면 길목 아님 (다음 구간에서 다시 보지 않게 seen엔 안 넣음)
        seen.insert(x);
        hits.push_back({k, x, sgn});
      }
    }
    // 길목과 겹치는 문은 뺌 — 같은 틈에 점이 넷이면 앞뒤로 오감 (문 가운데가 지난 길목 선분에서 0.4 m 안)
    hits.erase(std::remove_if(hits.begin(), hits.end(), [&](const Hit & h) {
      if (xs[h.x].gate) return false;
      for (const auto & g : hits) {
        if (!xs[g.x].gate) continue;
        const auto & G = xs[g.x];
        const double dx = G.bx - G.ax, dy = G.by - G.ay, len2 = dx * dx + dy * dy;
        const double u = std::clamp(((xs[h.x].cx - G.ax) * dx + (xs[h.x].cy - G.ay) * dy) / len2, 0.0, 1.0);
        if (std::hypot(xs[h.x].cx - G.ax - u * dx, xs[h.x].cy - G.ay - u * dy) < 0.4) return true;
      }
      return false;
    }), hits.end());
    const auto & r0 = path.front().pose.position, & g = goal.pose.position;
    auto point = [&](double px, double py, double yaw) {
      Pose p = goal;
      p.pose.position.x = px;
      p.pose.position.y = py;
      p.pose.orientation.x = p.pose.orientation.y = 0.0;
      p.pose.orientation.z = std::sin(yaw / 2);
      p.pose.orientation.w = std::cos(yaw / 2);
      return p;
    };
    auto add = [&](double px, double py, double yaw) {  // 바로 앞 점과 거의 같으면 (길목 B = 다음 길목 A) 한 번만
      if (!legs.empty() && std::hypot(legs.back().pose.position.x - px, legs.back().pose.position.y - py) < 0.15) return;
      legs.push_back(point(px, py, yaw));
    };
    double pcx = 0.0, pcy = 0.0, pfwd = -1.0;   // 앞 문 가운데 · 나가기 거리 (없으면 −1)
    bool pexit = false;                         // 앞 문 나가기 점을 넣었는지
    for (const auto & h : hits) {  // 경로 순서 (k 오름차순으로 찾았음)
      const auto & X = xs[h.x];
      const auto [cx, cy] = to_map(X.cx, X.cy);
      const double nx = h.sgn * (c * X.nx - s * X.ny), ny = h.sgn * (s * X.nx + c * X.ny), yaw = std::atan2(ny, nx);
      if (X.gate) {  // 사용자가 찍은 점 그대로 — 들어가는 점 · 나가는 점 (거꾸로면 B · A)
        auto [p1x, p1y] = to_map(h.sgn > 0 ? X.ax : X.bx, h.sgn > 0 ? X.ay : X.by);
        auto [p2x, p2y] = to_map(h.sgn > 0 ? X.bx : X.ax, h.sgn > 0 ? X.by : X.ay);
        if (std::hypot(r0.x - p1x, r0.y - p1y) > 0.25) add(p1x, p1y, yaw);  // 이미 그 점에 서 있으면 뺌
        if (std::hypot(g.x - p2x, g.y - p2y) > door_goal_skip_m_) add(p2x, p2y, yaw);
        RCLCPP_INFO(get_logger(), "길목 (%.2f, %.2f)→(%.2f, %.2f) 지남 — 경유 %zu개", p1x, p1y, p2x, p2y, legs.size());
        pfwd = -1.0;
        pexit = false;
        continue;
      }
      const double back = h.sgn > 0 ? X.d_neg : X.d_pos, fwd = h.sgn > 0 ? X.d_pos : X.d_neg;
      // 이미 들어가기 점에 서 있으면 뺌 — 11-15: 예전엔 문 가운데에서 back + 0.1 안이면 뺐는데, 문 옆 상자 자리(B4, 문에서 0.37 m)에서
      //   들어가기 점을 건너뛰고 대각선으로 문 모서리에 박았음
      const double inx = cx - nx * back, iny = cy - ny * back;
      // 붙은 문 둘 (11-15 시뮬 L2→L1: D8·D9 0.73 m — 앞 문 나가기 0.55 m가 다음 문 들어가기 0.55 m를 지나쳐 되돌아가다 끼임):
      //   두 문 사이가 나가기 + 들어가기보다 가까우면 앞 문 나가기 · 이 문 들어가기를 빼고 앞 문 들어가기에서 이 문 나가기로 곧게
      const bool chained = pfwd >= 0.0 && std::hypot(cx - pcx, cy - pcy) < pfwd + back + 0.05;
      if (chained) {
        if (pexit) legs.pop_back();
      } else if (std::hypot(r0.x - inx, r0.y - iny) > 0.15) {
        add(inx, iny, yaw);
      }
      pexit = std::hypot(g.x - cx, g.y - cy) > fwd + door_goal_skip_m_;
      if (pexit) legs.push_back(point(cx + nx * fwd, cy + ny * fwd, yaw));  // add 말고 — 다음 문이 붙으면 pop_back으로 뺌
      pcx = cx; pcy = cy; pfwd = fwd;
      RCLCPP_INFO(get_logger(), "문 (%.2f, %.2f) 지남 — 경유 %zu개", cx, cy, legs.size());
    }
    return legs;
  }

  // 뒤로 unstick_m_ 빼고 then (이 seq에서 한 번만) — 못 하면 false
  bool unstick(uint64_t seq, std::function<void()> then) {
    if (unstick_m_ <= 0.0 || !backup_->action_server_is_ready()) return false;
    unstuck_seq_ = seq;
    RCLCPP_WARN(get_logger(), "걸림 — %.2f m 뒤로 뺀 뒤 같은 점으로 다시", unstick_m_);
    BackUp::Goal b;
    b.target.x = unstick_m_;
    b.speed = 0.05f;
    b.time_allowance.sec = 10;
    // costmap 검사 끔 (11-16 실기·시뮬: 벽·상자에 붙어 걸린 자리에서 뒤쪽 costmap이 '충돌 앞'으로 막아 못 빠져나옴)
    //   0.15 m · 0.05 m/s만 — 뒤는 collision_monitor backward 반원이 계속 지킴
    b.disable_collision_checks = true;
    rclcpp_action::Client<BackUp>::SendGoalOptions bo;
    bo.result_callback = [this, seq, then](const rclcpp_action::ClientGoalHandle<BackUp>::WrappedResult &) {
      if (seq == nav_seq_) then();
    };
    backup_->async_send_goal(b, bo);
    return true;
  }

  // 가르친 경로 → 경유점 (map). 로봇 자리 = 플래너 경로 첫 점. 없거나 안 맞으면 빈 목록
  std::vector<Pose> route_legs(const Pose & robot, const Pose & goal) {
    std::vector<Pose> legs;
    std::error_code ec;
    if (routes_dir_.empty() || !std::filesystem::is_directory(routes_dir_, ec)) return legs;
    const double c0 = std::cos(-start_[2]), s0 = std::sin(-start_[2]);  // arena → map (문과 같은 식, 보정은 로봇 위치에 들어감)
    std::vector<std::vector<Pt>> routes;
    std::vector<std::string> names;
    for (const auto & e : std::filesystem::directory_iterator(routes_dir_, ec)) {
      if (e.path().extension() != ".txt") continue;
      auto pts = load_route(e.path().string());
      for (auto & p : pts) {
        const double ax = p.first - start_[0], ay = p.second - start_[1];
        p = {c0 * ax - s0 * ay, s0 * ax + c0 * ay};
      }
      routes.push_back(std::move(pts));
      names.push_back(e.path().stem().string());
    }
    const Pt r{robot.pose.position.x, robot.pose.position.y}, g{goal.pose.position.x, goal.pose.position.y};
    const auto pts = pick_route(routes, r, g, route_start_m_, route_goal_m_, route_tol_m_);
    for (size_t i = 0; i < pts.size(); ++i) {
      const Pt nxt = i + 1 < pts.size() ? pts[i + 1] : g;  // 다음 점을 보는 방향
      const double yaw = std::atan2(nxt.second - pts[i].second, nxt.first - pts[i].first);
      Pose p = goal;
      p.pose.position.x = pts[i].first;
      p.pose.position.y = pts[i].second;
      p.pose.orientation.x = p.pose.orientation.y = 0.0;
      p.pose.orientation.z = std::sin(yaw / 2);
      p.pose.orientation.w = std::cos(yaw / 2);
      legs.push_back(p);
    }
    if (!routes.empty())
      RCLCPP_INFO(get_logger(), "가르친 경로 %zu개 중 %s — 경유 %zu개", routes.size(), legs.empty() ? "맞는 것 없음 (플래너)" : "맞음", legs.size());
    return legs;
  }

  // legs[i]로 NavigateToPose — 성공하면 다음 점, 마지막 점에만 look · near (경유점은 그대로 도착해야 함)
  void drive(std::shared_ptr<std::vector<Pose>> legs, size_t i, std::set<std::string> flags, uint64_t seq, Done done, int tries = 0) {
    if (seq != nav_seq_) return;
    const bool last = i + 1 == legs->size();
    NavigateToPose::Goal goal;
    goal.pose = (*legs)[i];
    // 시각 0 = Nav2가 가장 최근 TF로 옮김 — now()면 TF가 늦게 오는 순간(젯슨 부하) 컨트롤러가 목표를 costmap(odom)으로 못 옮겨
    //   error 102 'Unable to transform goal pose into costmap frame'로 출발 0.4 s 만에 실패 (11-15 실기 S3)
    goal.pose.header.stamp = builtin_interfaces::msg::Time();
    stage(std::string(last ? "Nav2 (" : "Nav2 경유 (") + std::to_string(goal.pose.pose.position.x).substr(0, 5) + ", " +
          std::to_string(goal.pose.pose.position.y).substr(0, 5) + ")");
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
    auto finished = std::make_shared<bool>(false);  // 감시가 먼저 끝내면 취소 뒤 오는 결과는 무시
    auto watch = std::make_shared<ProgressWatch>(stall_s_, stall_gain_m_);
    auto look = std::make_shared<bool>(last && flags.count("look") > 0);  // true = 아직 팔을 안 올림
    const bool near = last && flags.count("near") > 0;
    auto left = std::make_shared<double>(1e9);  // 목표까지 직선 거리 (마지막 피드백)
    auto yaw_err = std::make_shared<double>(M_PI);  // 목표 방향과 차 [rad] (마지막 피드백)
    auto here = std::make_shared<Pose>();  // 로봇 자세 (마지막 피드백) — 끼임 풀기 회전 기준
    const auto gp = goal.pose.pose.position;
    const double gyaw = yaw_of(goal.pose.pose.orientation);
    // near: 끝날 때 목표 근처면 성공으로 바꿈 · 경유점 성공이면 다음 점
    auto finish = [this, near, left, yaw_err, here, done, legs, i, last, flags, seq, tries](bool ok, const std::string & why) {
      if (!ok && near && *left <= near_m_ && *yaw_err <= near_yaw_) {
        RCLCPP_WARN(get_logger(), "%s — 목표 %.2f m · %.0f° 앞이라 성공 처리 (near, 나머지는 approach)", why.c_str(), *left,
                    *yaw_err * 180.0 / M_PI);
        ok = true;
      }
      if (ok && !last) { drive(legs, i + 1, flags, seq, done); return; }
      if (!ok && why == "nav_no_progress" && tries < unstuck_tries_ && seq == nav_seq_) {
        unstuck(*here, (*legs)[i], seq, [this, legs, i, flags, seq, done, tries]() { drive(legs, i, flags, seq, done, tries + 1); });
        return;
      }
      done(ok, ok ? "" : why);
    };
    if (stall_s_ > 0.0 || *look || near) {
      opt.feedback_callback = [this, finished, watch, look, left, yaw_err, here, gp, gyaw, finish](
          rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr h, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        if (*finished) return;
        *here = fb->current_pose;
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
    opt.result_callback = [this, seq, finished, finish, legs, i, flags, done](
        const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
      if (seq == nav_seq_) nav_goal_.reset();
      if (*finished || seq != nav_seq_) return;
      *finished = true;
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
      // 출발 칸이 막힘(205 START_OCCUPIED) — 한 번만 뒤로 빼고 같은 점으로 다시
      if (!ok && r.result && r.result->error_code == 205 && unstuck_seq_ != seq &&
          unstick(seq, [this, legs, i, flags, seq, done]() { drive(legs, i, flags, seq, done); })) {
        RCLCPP_WARN(get_logger(), "출발 칸이 막힘 (205) — 뒤로 뺀 뒤 다시");
        return;
      }
      finish(ok, ok ? "" : "nav_failed");
    };
    nav_->async_send_goal(goal, opt);
  }

  // 경유점 + 목표(legs 끝)를 NavigateThroughPoses 한 번으로 — 경유점에서 서지 않음. 끝 처리는 drive()의 마지막 점과 같음:
  //   near(목표 nav_near_m · nav_near_yaw 안이면 실패도 성공) · look(목표까지 nav_look_m 안이면 팔) · 진척 감시 · 끼임 풀기
  //   끼임 풀기 뒤에는 아직 안 지난 점(피드백 number_of_poses_remaining)부터 다시
  void drive_through(std::shared_ptr<std::vector<Pose>> legs, std::set<std::string> flags, uint64_t seq, Done done, int tries = 0) {
    if (seq != nav_seq_) return;
    if (legs->size() < 2) { drive(legs, 0, flags, seq, done, tries); return; }
    NavigateThrough::Goal goal;
    goal.poses.header.frame_id = legs->back().header.frame_id;
    for (auto p : *legs) {
      p.header.stamp = builtin_interfaces::msg::Time();  // 0 = 가장 최근 TF (drive 설명 — error 102)
      goal.poses.goals.push_back(p);
    }
    const auto gp = legs->back().pose.position;
    const double gyaw = yaw_of(legs->back().pose.orientation);
    stage("Nav2 경유 " + std::to_string(legs->size() - 1) + "개 지나 (" + std::to_string(gp.x).substr(0, 5) + ", " +
          std::to_string(gp.y).substr(0, 5) + ")");
    RCLCPP_INFO(get_logger(), "경유점 %zu개를 서지 않고 지나 목표로 (NavigateThroughPoses)", legs->size() - 1);
    rclcpp_action::Client<NavigateThrough>::SendGoalOptions opt;
    auto finished = std::make_shared<bool>(false);
    auto watch = std::make_shared<ProgressWatch>(stall_s_, stall_gain_m_);
    auto look = std::make_shared<bool>(flags.count("look") > 0);
    const bool near = flags.count("near") > 0;
    auto left = std::make_shared<double>(1e9);
    auto yaw_err = std::make_shared<double>(M_PI);
    auto here = std::make_shared<Pose>();
    auto remain = std::make_shared<size_t>(legs->size());  // 아직 안 지난 점 수 (목표 포함)
    auto finish = [this, near, left, yaw_err, here, remain, done, legs, flags, seq, tries](bool ok, const std::string & why) {
      if (!ok && near && *left <= near_m_ && *yaw_err <= near_yaw_) {
        RCLCPP_WARN(get_logger(), "%s — 목표 %.2f m · %.0f° 앞이라 성공 처리 (near, 나머지는 approach)", why.c_str(), *left,
                    *yaw_err * 180.0 / M_PI);
        ok = true;
      }
      if (!ok && why == "nav_no_progress" && tries < unstuck_tries_ && seq == nav_seq_) {
        const size_t n = std::clamp<size_t>(*remain, 1, legs->size());
        auto rest = std::make_shared<std::vector<Pose>>(legs->end() - static_cast<std::ptrdiff_t>(n), legs->end());
        unstuck(*here, rest->front(), seq, [this, rest, flags, seq, done, tries]() { drive_through(rest, flags, seq, done, tries + 1); });
        return;
      }
      done(ok, ok ? "" : why);
    };
    opt.feedback_callback = [this, finished, watch, look, left, yaw_err, here, remain, gp, gyaw, finish](
        rclcpp_action::ClientGoalHandle<NavigateThrough>::SharedPtr h, const std::shared_ptr<const NavigateThrough::Feedback> fb) {
      if (*finished) return;
      *here = fb->current_pose;
      if (fb->number_of_poses_remaining > 0) *remain = static_cast<size_t>(fb->number_of_poses_remaining);
      *left = std::hypot(fb->current_pose.pose.position.x - gp.x, fb->current_pose.pose.position.y - gp.y);
      *yaw_err = std::abs(std::remainder(yaw_of(fb->current_pose.pose.orientation) - gyaw, 2 * M_PI));
      // distance_remaining은 남은 경유점까지 합친 길이 — look은 마지막 구간(남은 점 = 목표 하나)에서만
      if (*look && *remain <= 1 && fb->distance_remaining > 0.01 && fb->distance_remaining < look_m_) {
        *look = false;
        if (arm_.moveit_ready())
          arm_.move_joints(look_q_, [this](bool ok) { if (!ok) RCLCPP_WARN(get_logger(), "주행 중 look 자세 실패"); });
        else RCLCPP_WARN(get_logger(), "MoveIt 없음 — 주행 중 look 자세 건너뜀");
      }
      if (stall_s_ <= 0.0 || !watch->update(now().seconds(), fb->distance_remaining)) return;
      *finished = true;
      RCLCPP_WARN(get_logger(), "진척 없음 — 남은 %.2f m가 %.0f s 동안 %.2f m도 안 줄음 (회복 %d회 · 남은 점 %d) → 취소",
                  fb->distance_remaining, stall_s_, stall_gain_m_, fb->number_of_recoveries, fb->number_of_poses_remaining);
      through_->async_cancel_goal(h);
      finish(false, "nav_no_progress");
    };
    opt.goal_response_callback = [this, seq, finished, finish](const rclcpp_action::ClientGoalHandle<NavigateThrough>::SharedPtr & h) {
      if (seq != nav_seq_) {
        if (h) through_->async_cancel_goal(h);
        return;
      }
      if (h) { through_goal_ = h; return; }
      if (*finished) return;
      *finished = true;
      finish(false, "nav_rejected");
    };
    opt.result_callback = [this, seq, finished, finish](const rclcpp_action::ClientGoalHandle<NavigateThrough>::WrappedResult & r) {
      if (seq == nav_seq_) through_goal_.reset();
      if (*finished || seq != nav_seq_) return;
      *finished = true;
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
      finish(ok, ok ? "" : "nav_failed");
    };
    through_->async_send_goal(goal, opt);
  }

  // 끼임 풀기: 곧게 후진 → 다시 받은 경로의 0.4 m 앞 쪽으로 제자리 회전 → then(). 어느 단계가 실패해도 다음으로 넘어감
  //   (후진이 뒤 벽에 막혀도 회전 · 재출발은 해 봄). 후진은 costmap 충돌 검사를 끔 — 이미 팽창 칸 안이라 거절됨 (collision_monitor는 그대로)
  void unstuck(const Pose & here, const Pose & target, uint64_t seq, std::function<void()> then) {
    RCLCPP_WARN(get_logger(), "끼임 풀기 — 후진 %.2f m → 경로 쪽으로 회전 → 다시 출발", unstuck_back_m_);
    stage("끼임 풀기: 후진");
    const double yaw0 = yaw_of(here.pose.orientation);
    auto turn = [this, target, seq, then, yaw0]() {
      if (seq != nav_seq_) return;
      if (!path_->action_server_is_ready() || !spin_->action_server_is_ready()) { then(); return; }
      ComputePath::Goal pg;
      pg.goal = target;
      pg.goal.header.stamp = builtin_interfaces::msg::Time();
      pg.use_start = false;
      rclcpp_action::Client<ComputePath>::SendGoalOptions po;
      po.result_callback = [this, seq, then, yaw0](const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult & r) {
        if (seq != nav_seq_) return;
        if (r.code != rclcpp_action::ResultCode::SUCCEEDED || !r.result || r.result->path.poses.size() < 2) { then(); return; }
        const auto & ps = r.result->path.poses;
        const auto & p0 = ps.front().pose.position;
        auto ahead = ps.back().pose.position;
        for (const auto & p : ps)
          if (std::hypot(p.pose.position.x - p0.x, p.pose.position.y - p0.y) >= 0.4) { ahead = p.pose.position; break; }
        const double dyaw = std::remainder(std::atan2(ahead.y - p0.y, ahead.x - p0.x) - yaw0, 2 * M_PI);
        if (std::abs(dyaw) < 0.15) { then(); return; }
        stage("끼임 풀기: 회전");
        Spin::Goal sg;
        sg.target_yaw = static_cast<float>(dyaw);
        sg.time_allowance.sec = 10;
        rclcpp_action::Client<Spin>::SendGoalOptions so;
        so.result_callback = [this, seq, then](const rclcpp_action::ClientGoalHandle<Spin>::WrappedResult &) {
          if (seq == nav_seq_) then();
        };
        spin_->async_send_goal(sg, so);
      };
      po.goal_response_callback = [this, seq, then](const rclcpp_action::ClientGoalHandle<ComputePath>::SharedPtr & h) {
        if (!h && seq == nav_seq_) then();
      };
      path_->async_send_goal(pg, po);
    };
    if (!backup_->action_server_is_ready()) { turn(); return; }
    BackUp::Goal bg;
    bg.target.x = unstuck_back_m_;
    bg.speed = 0.05f;
    bg.time_allowance.sec = static_cast<int32_t>(unstuck_back_m_ / 0.05 + 4);
    bg.disable_collision_checks = true;
    rclcpp_action::Client<BackUp>::SendGoalOptions bo;
    bo.result_callback = [turn](const rclcpp_action::ClientGoalHandle<BackUp>::WrappedResult &) { turn(); };
    bo.goal_response_callback = [turn](const rclcpp_action::ClientGoalHandle<BackUp>::SharedPtr & h) { if (!h) turn(); };
    backup_->async_send_goal(bg, bo);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
  rclcpp_action::Client<NavigateThrough>::SharedPtr through_;
  rclcpp_action::Client<ComputePath>::SharedPtr path_;
  rclcpp_action::Client<Spin>::SharedPtr spin_;
  rclcpp_action::Client<Drive>::SharedPtr drive_;
  rclcpp_action::ClientGoalHandle<Spin>::SharedPtr spin_goal_;
  rclcpp_action::ClientGoalHandle<Drive>::SharedPtr drive_goal_;
  rclcpp_action::Client<Follow>::SharedPtr follow_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_;
  uint64_t unstuck_seq_ = 0;
  double max_end_err_ = 0.6, unstick_m_ = 0.15;
  rclcpp_action::ClientGoalHandle<Follow>::SharedPtr follow_goal_;
  std::string route_mode_ = "follow";
  bool anchor_ = true;
  bool taught_ = false;
  double dev_m_ = 0.30, dev_rad_ = 0.26, route_speed_ = 0.15, fix_rad_ = 0.052, settle_s_ = 0.8;
  bool fallback_nav2_ = false;
  int unstuck_tries_ = 2;
  double unstuck_back_m_ = 0.15;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr corr_sub_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_goal_;  // 지금 보낸 Nav2 goal (stop이 이것만 취소)
  rclcpp_action::ClientGoalHandle<NavigateThrough>::SharedPtr through_goal_;  // 경유점 지도 길 goal (via_through)
  uint64_t nav_seq_ = 0;  // 보낸 goal 번호 — stop하면 올라가 옛 응답·결과를 가려냄
  template <typename... A>
  static std::string fmt(const char * f, A... a) {
    char buf[160];
    std::snprintf(buf, sizeof buf, f, a...);
    return buf;
  }
  static double yaw_of(const geometry_msgs::msg::Quaternion & q) {
    return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }
  double stall_s_ = 25.0, stall_gain_m_ = 0.10, look_m_ = 0.6, near_m_ = 0.15, near_yaw_ = 0.8, door_goal_skip_m_ = 0.30;
  std::vector<double> doors_, gates_, no_path_, start_;
  std::map<std::string, double> corr_{{"dx", 0.0}, {"dy", 0.0}, {"dth", 0.0}};
  bool door_use_corr_ = false, via_through_ = true;
  std::string routes_dir_;
  double route_start_m_ = 0.5, route_goal_m_ = 0.4, route_tol_m_ = 0.10;
  ViaGraph via_;
  ArmIO arm_;
  std::vector<double> look_q_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::NavigateSkill)
