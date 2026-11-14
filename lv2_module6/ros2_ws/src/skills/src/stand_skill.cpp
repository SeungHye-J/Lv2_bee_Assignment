// skill_stand — 누운 퍽 세우기와 그 자리에서 다시 쥐기 (팀원 right_env "tilt", 경유점: stand_plan.hpp — stand_waypoints ik | table)
//   /skills/stand (goal.pose = 누운 퍽 map 자세 · 긴 축 = orientation · goal.shape · goal.tag):
//     벌림(stand_open_m) → MoveIt으로 먼 끝 위 → 멈춤 → 내려감 → 위치형 쥐기: 목표 stand_pinch_m을 닫는 전류로 → 쥠 확인 → 쥐는 전류로 낮춤 →
//     대각선 끌기(stand_drag_m > 0, 실기 방식) 또는 호 · 멈춤 · 눌러 놓음 → 멈춤 → 벌림 → 위로. 손은 놓고 끝 (경유점을 기억 — 다음 stand_pick)
//   /skills/stand_pick: 직전 stand의 경유점에서 같은 손목으로 선 퍽 위 → 바로 아래로 → 멈춤 → 제 힘으로 닫기 → 잡기 판정 → 들어올림
//     (reset_sag 안 함 — 세운 직후 처짐 보정을 0으로 되돌리면 손이 처진 만큼 떨어짐)
//   정책: models/skills/regrasp_after_stand/ (동료 regrasp_stood = 세우기 + 다시 쥐기 한 에피소드) — 로봇 입력 모양(stand)이
//     rl_arm_correction에 생기기 전까지 meta use: false (기준 동작만)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "ml/mlp_policy.hpp"
#include "perception/msg/hand_relative.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/skill_node.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace skills {

namespace {
constexpr double STAND_PINCH_S = 0.3, STAND_SETTLE_S = 0.3, STAND_OPEN_S = 0.3, REGRASP_CLOSE_S = 0.5;
constexpr double STAND_HANG_S = 0.5;  // 호 안 멈춤
// stand_grasp 정책 구간 — isaac stand_grasp_env와 같은 값 (hold_s 0.5 · 내려가기 vel_scale 0.3 = arm_io CONTACT_VS)
constexpr double STAND_GRASP_HOLD_S = 0.5, STAND_GRASP_VS = 0.3;
}  // namespace

class StandSkill : public SkillNode {
public:
  explicit StandSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_stand", options), arm_(*this) {
    waypoints_ = declare_parameter<std::string>("stand_waypoints", "ik");
    box_top_m_ = declare_parameter<double>("stand_box_top_m", 0.07);           // 시작 박스 윗면 높이 — 누운 퍽 바닥
    open_m_ = declare_parameter<double>("stand_open_m", 0.012);                // [m] 벌림 (손가락 하나)
    pinch_m_ = declare_parameter<double>("stand_pinch_m", 0.006);              // [m] 쥐는 목표 — 힘은 전류 상한과 이 위치 오차로 정함
                                                                               //   (11/08: 0.007은 30 mm 퍽 닿는 자리 0.0062보다 넓어 안 쥠 → 0.006)
    close_a_ = declare_parameter<double>("stand_close_current_a", 0.01614);    // [A] 목표까지 닫을 때
    pinch_a_ = declare_parameter<double>("stand_pinch_current_a", 0.00538);    // [A] 닫은 뒤 낮춰 쥐는 전류
    grip_a_ = declare_parameter<double>("stand_grip_current_a", 0.007);        // [A] 이 전류 이상 + 안 움직임 = 쥠
    empty_m_ = declare_parameter<double>("stand_empty_m", 0.003);              // [m] 이보다 닫히면 빈손
    grip_timeout_s_ = declare_parameter<double>("stand_grip_timeout_s", 4.0);
    hold_s_ = declare_parameter<double>("stand_hold_s", 0.3);  // [s] 퍽 위(stand 첫 자세 · stand_pick 쥐는 자리)에서 멈춤 — 처짐 보정 쌓임
    sp_.drag_m = declare_parameter<double>("stand_drag_m", sp_.drag_m);
    sp_.drag_lift_m = declare_parameter<double>("stand_drag_lift_m", sp_.drag_lift_m);
    sp_.arc_end_deg = declare_parameter<double>("stand_arc_end_deg", sp_.arc_end_deg);
    sp_.set_dz_m = declare_parameter<double>("stand_set_dz_m", sp_.set_dz_m);
    sp_.tilt_over_m = declare_parameter<double>("stand_tilt_over_m", 0.030);   // 호 끝까지 더 올림 (시뮬 0.010 · 실기 11/08 0.030)
    sp_.flip_roll = declare_parameter<bool>("stand_flip_roll", false);  // 손목 roll 180° — gripper_cam이 반대쪽(퍽 몸통 쪽)에 옴
    sp_.arc_return = declare_parameter<bool>("stand_arc_return", false);       // 호 끝에서 90°로 되돌리지 않고 바로 벌림 (실기 11/08)
    sp_.above_m = declare_parameter<double>("stand_above_m", sp_.above_m);
    sp_.lift_away_m = declare_parameter<double>("stand_lift_away_m", sp_.lift_away_m);
    sp_.contact_h_box = declare_parameter<double>("stand_contact_h_box_m", sp_.contact_h_box);
    sp_.contact_h_cyl = declare_parameter<double>("stand_contact_h_cyl_m", sp_.contact_h_cyl);
    vs_ = declare_parameter<double>("stand_vel_scale", 0.3);                   // 내려가기·위로 × 팀 관절 한계
    arc_vs_ = declare_parameter<double>("stand_arc_vel_scale", 0.5);           // 호 × 팀 관절 한계
    table_.yaw_deg = declare_parameter<std::vector<double>>("stand_table_yaw_deg", std::vector<double>{});
    table_.ycs = declare_parameter<std::vector<double>>("stand_table_ycs", std::vector<double>{});
    table_.box_cx = declare_parameter<double>("stand_table_box_cx", 0.0);
    table_.box_h = declare_parameter<double>("stand_table_box_h", 0.0);
    table_.w = static_cast<int>(declare_parameter<int64_t>("stand_table_w", 0));
    table_.q[0] = declare_parameter<std::vector<double>>("stand_table_cyl_q", std::vector<double>{});
    table_.P[0] = declare_parameter<std::vector<double>>("stand_table_cyl_p", std::vector<double>{});
    table_.q[1] = declare_parameter<std::vector<double>>("stand_table_box_q", std::vector<double>{});
    table_.P[1] = declare_parameter<std::vector<double>>("stand_table_box_p", std::vector<double>{});
    std::string why;
    if (waypoints_ != "ik" && waypoints_ != "table") { RCLCPP_WARN(get_logger(), "stand_waypoints '%s' 모름 — ik로", waypoints_.c_str()); waypoints_ = "ik"; }
    if (waypoints_ == "table" && !table_.ready(why))
      RCLCPP_WARN(get_logger(), "stand_waypoints table인데 표가 없음 (params/stand_table.yaml) — stand는 stand_table_missing으로 실패");
    policy_ = load_model("regrasp_after_stand");
    grasp_model_ = load_model("stand_grasp");  // 누운 퍽 모서리 쥐기 보정 (isaac stand_grasp_env, layout pick)
    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", rclcpp::QoS(10));
    declare_parameter<double>("stand_end_margin_m", 0.03);   // [m] 선 퍽 가장자리가 상자 가장자리에서 이보다 가까우면 반대 끝을 잡아 반대로 세움 (0 = 안 바꿈)
    declare_parameter<double>("stand_score_margin", 3.0);    // 쥘 끝 점수 가중치: 선 자리 여유 (choose_end)
    declare_parameter<double>("stand_score_slack", 1.0);     //   관절 한계 여유 (해가 넓은 쪽)
    declare_parameter<double>("stand_score_cam", 2.0);       //   stand_flip_roll대로 (카메라가 퍽 몸통 쪽)
    declare_parameter<double>("stand_score_far", 0.2);       //   먼 끝 (동점이면 기존대로)
    objects_sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
      [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { objects_ = m; });
    declare_parameter<double>("stand_puck_dy_m", 0.0);         // [m] inspect 퍽 위치에 더함 (base_footprint y, 왼쪽 +) — 손가락 사이 가운데 맞추기 (11-15)
    declare_parameter<double>("stand_pinch_from_end_m", 0.0);  // [m] 쥘 끝에서 퍽 안쪽으로 이만큼 들어와 쥠 (퍽 축 방향) — 0 = 끝 모서리
    declare_parameter<double>("stand_puck_dx_m", -0.010);      // [m] inspect 퍽 위치에 더함 (base_footprint x) — 실기 11/08 inspect가 손 카메라보다 10 mm 앞
    declare_parameter<double>("stand_arc_k", 1.0);           // 호의 퍽 축 방향 배율 (1 원 · 0 그대로 위로)
    declare_parameter<double>("stand_release_deg", -1.0);
    // 먼 누운 퍽: 세우기 대신 로봇 쪽 끝을 쥐고 로봇 쪽으로 끌어옴 (11-15 실기: 0.287 m 퍽은 세우기 IK가 안 풀리고, 먼 자리에서
    //   세우면 상자 밖으로 떨어짐). 가운데가 base_footprint에서 stand_pull_min_m보다 멀거나 세우기 IK가 안 풀리면 →
    //   stand_pull_to_m까지 끌고(3~10 cm) 놓은 뒤 성공으로 끝 (계획 쪽이 look에서 아직 누움을 보고 다시 stand). 0 = 끔
    // 쥠 확인 뒤 호를 도는 동안의 목표 [m] — stand_pinch_m(0.0050, 퍽 안쪽)은 빈손을 가리려고 퍽을 누르는 목표라, 확인 뒤엔
    //   퍽 폭(0.0062 = 간격 30 mm)으로 풀어 낮은 전류(stand_pinch_current_a)로만 받침 → 퍽이 손가락 사이에서 돎 (11-15 사용자: 세게 쥠)
    declare_parameter<double>("stand_hold_m", 0.0062);
    declare_parameter<double>("stand_pull_min_m", 0.26);
    // 거리에 따라 쥐는 높이 보정 [m/m]: (가운데 거리 − stand_contact_ref_m) × 이 값만큼 손가락을 낮춤 (±10 mm 상한) · 매 goal마다 읽음
    declare_parameter<double>("stand_contact_dz_per_m", 0.0625);
    declare_parameter<double>("stand_contact_ref_m", 0.20);
    declare_parameter<double>("stand_pull_to_m", 0.20);
    declare_parameter<double>("stand_pull_vel_scale", 0.2);  // 끌기 속도 × 팀 관절 한계 (천천히 — 퍽이 손에서 빠지지 않게)   // [deg] 호가 이 각도를 지날 때 그리퍼를 벌림 (팔은 호를 계속) · ≤ 0 = 호 끝에서 (기존)
    // 호 방식: 내려가기 전 멈춰 손 카메라로 퍽을 다시 재고 경유점을 다시 풂 — 쥐는 높이 근처는 카메라–퍽 10 cm 안이라 depth가 비어서
    declare_parameter<bool>("stand_cam", true);
    declare_parameter<double>("stand_cam_above_m", 0.06);    // [m] 잴 때 손끝 높이 (쥐는 자리 위) — 6 cm면 카메라–퍽 10 cm 넘음
    declare_parameter<double>("stand_cam_back_m", 0.025);    // [m] 잴 때 쥐는 자리에서 로봇 쪽(퍽 가까운 끝 쪽)으로 — 퍽 전체가 화면에 들어오게
    declare_parameter<double>("stand_cam_wait_s", 1.2);      // [s] 재는 시간 (손 카메라 검출 ≤ 4 Hz)
    declare_parameter<int>("stand_cam_min_n", 2);            // 이만큼 못 보면 inspect 값 그대로
    declare_parameter<double>("stand_cam_max_m", 0.03);      // [m] inspect 자리에서 이보다 멀리는 안 옮김 (잘못 본 퍽)
    declare_parameter<double>("stand_cam_max_deg", 30.0);    // 축도 이보다 많이는 안 돌림
    hr_sub_ = create_subscription<perception::msg::HandRelative>("/hand_relative", rclcpp::QoS(10),
      [this](perception::msg::HandRelative::ConstSharedPtr m) { if (m->kind == "puck") hr_ = m; });
    serve("stand", [this](const SkillGoal & g, Done done) {
      when_ready([this]() { return arm_.moveit_ready() && arm_.gripper_ready(); }, "arm_unavailable", done,
                 [this, g, done]() { stand(g, done); });
    });
    serve("stand_pick", [this](const SkillGoal &, Done done) {
      when_ready([this]() { return arm_.moveit_ready() && arm_.gripper_ready(); }, "arm_unavailable", done,
                 [this, done]() { stand_pick(done); });
    });
  }

protected:
  void stop() override {
    if (cam_timer_) cam_timer_->cancel();
    if (release_timer_) release_timer_->cancel();
    arm_.stop();
  }

private:
  void stand(const SkillGoal & g, Done done) {
    refresh_params();  // 실행 중 ros2 param set /skill_stand stand_… 이 다음 goal부터 먹게
    // 퍽 (map) → 로봇 root (base_footprint): 가운데 xy · 긴 축 yaw (180° 대칭 → 로봇에서 먼 쪽으로)
    tf2::Transform t;
    try {
      const auto m = arm_.tf().lookupTransform("base_footprint", g.pose.header.frame_id.empty() ? "map" : g.pose.header.frame_id,
                                               tf2::TimePointZero).transform;
      t = tf2::Transform(tf2::Quaternion(m.rotation.x, m.rotation.y, m.rotation.z, m.rotation.w),
                         tf2::Vector3(m.translation.x, m.translation.y, m.translation.z));
    } catch (const std::exception &) {
      done(false, "stand_no_tf");
      return;
    }
    const auto & pp = g.pose.pose.position;
    const auto & q = g.pose.pose.orientation;
    const tf2::Vector3 c = t * tf2::Vector3(pp.x, pp.y, pp.z);
    const tf2::Vector3 ax = t.getBasis() * tf2::Vector3(1.0 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 0.0);
    double yaw = stand::away_yaw(std::atan2(ax.y(), ax.x()));
    const stand::V3 centre = {c.x() + get_parameter("stand_puck_dx_m").as_double(),
                              c.y() + get_parameter("stand_puck_dy_m").as_double(), box_top_m_};
    const bool is_box = g.shape != "cyl";
    {  // 멀수록 더 깊게: 손가락 높이를 (가운데 거리 − stand_contact_ref_m) × stand_contact_dz_per_m만큼 낮춤 (가까우면 올림)
      //   11-15 실기: 먼 퍽일수록 얕게 쥐는 느낌 (사용자) — 팔을 뻗을수록 실제 손이 TF보다 높게 감. 0 = 끔
      const double k = get_parameter("stand_contact_dz_per_m").as_double(), ref = get_parameter("stand_contact_ref_m").as_double();
      // 11-15 실측 (goto --pose x 0.10, 손 수직): 손끝이 상자 위 0.16 m 25 · 0.20 m 25 · 0.28 m 30 mm (계획 30) — 0.20 m 안은 같고
      //   그 너머로 8 cm에 5 mm 높아짐 → 0.20 m 너머만 0.0625 m/m로 낮춤 (가까운 쪽은 안 바꿈)
      const double dz = std::clamp(k * std::max(0.0, std::hypot(centre[0], centre[1]) - ref), 0.0, 0.010);
      sp_.contact_h_box -= dz;
      sp_.contact_h_cyl -= dz;
      if (std::abs(dz) > 0.0005) RCLCPP_INFO(get_logger(), "stand: 거리 보정 — 손가락 높이 %+.1f mm", -dz * 1000);
    }
    std::vector<stand::Q5> qs;
    std::string why;
    const bool ok = waypoints_ == "table" ? stand::plan_table(table_, centre, yaw, is_box, qs, why)
                                          : stand::plan_ik(centre, yaw, is_box, sp_, qs, why);
    if (waypoints_ == "ik") choose_end(centre, is_box, ok, yaw, qs);
    const double pull_min = get_parameter("stand_pull_min_m").as_double(), dist = std::hypot(centre[0], centre[1]);
    if (waypoints_ == "ik" && pull_min > 0.0 && (dist > pull_min || qs.empty())) {
      pull(centre, yaw, is_box, dist, done);
      return;
    }
    if (qs.empty()) { done(false, why); return; }
    if (waypoints_ == "table" && std::abs(centre[0] - table_.box_cx) > 0.02)
      RCLCPP_WARN(get_logger(), "stand 표: 퍽이 표 기준 x에서 %.0fmm — dock 자리 확인", (centre[0] - table_.box_cx) * 1000);
    stage(std::string(is_box ? "사각" : "원기둥") + " 퍽 (" + std::to_string(centre[0]).substr(0, 5) + ", " +
          std::to_string(centre[1]).substr(0, 5) + ") 축 " + std::to_string(static_cast<int>(yaw * 180 / M_PI)) + "° — 경유점 " +
          waypoints_ + " " + std::to_string(qs.size()) + "개");
    done_qs_.clear();
    arm_.clear_flags();
    arm_.reset_sag();
    const double open = open_m_, pinch = pinch_m_, hold_m = get_parameter("stand_hold_m").as_double();
    const double vs = vs_, avs = arc_vs_;
    const stand::Params sp = waypoints_ == "table" ? stand::Params{} : sp_;  // 표는 right_env 호 그대로
    // 경유점은 손 카메라로 다시 잰 뒤 바뀔 수 있어서 뒤 단계들은 run->qs를 실행 때 읽음
    auto run = std::make_shared<SRun>();
    run->qs = qs;
    run->centre0 = centre;
    run->yaw0 = yaw;
    run->is_box = is_box;
    const auto wp = [run]() { return std::vector<stand::Q5>(run->qs.begin(), run->qs.end() - 2); };  // [위 · 시작 · 호 … · 놓기 · 위로] (뒤 2개 = 다시 잡기)
    stand::Q5 look;
    const bool cam = get_parameter("stand_cam").as_bool() && waypoints_ == "ik" && look_pose(centre, yaw, is_box, qs.front(), look);
    const stand::Q5 first = cam ? look : qs.front();
    std::vector<ArmIO::Step> st = {
      arm_.gripper_step(open),
      arm_.joints_step(std::vector<double>(first.begin(), first.end())),  // 어디서든 MoveIt 계획으로 (충돌 확인)
      arm_.wait_step(hold_s_)};
    if (cam) {
      const auto & gp = g.pose.pose.position;
      st.push_back(cam_step(run, g.tag, gp.x, gp.y, gp.z));
      st.push_back(arm_.traj_step([run, vs](const stand::Q5 & now) { return stand::sample_move(now, run->qs.front(), vs, TRAJ_DT); }));
    }
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(st);
    // stand_grasp 정책 (model/skills/stand_grasp, use: true): 위 멈춤·내려가기만 학습 보정 (isaac stand_grasp_env) — 시뮬과
    // 같은 단계 신호 (0 위 · 1 내려가기 · 3 쥐기 · 4 원호 · 5 그 뒤), 내려가기는 contact 속도, 쥐는 순간부터 Δ 고정 (원호 내내 유지)
    const bool learned = grasp_model_.use && waypoints_ == "ik";
    const auto gm = std::make_shared<SkillModel>(grasp_model_);
    const auto ctx = [this, run, gm](uint8_t phase, bool holding, bool live) -> ArmIO::Step {
      return [this, run, gm, phase, holding, live](ArmIO::Cb c) { arm_.skill_ctx(gm.get(), phase, holding, live, run->qs[0], run->qs[1]); c(true); };
    };
    if (learned) {
      // hand_relative rel = 계획한 쥘 끝 − 손 (= 본 퍽 가운데 − 손 + 계획한 (쥘 끝 − 가운데)): map 좌표로 넘김
      const tf2::Transform inv = t.inverse();
      const tf2::Vector3 pc = inv * tf2::Vector3(centre[0], centre[1], box_top_m_ + stand::PUCK_W / 2);
      const tf2::Vector3 pe = inv * tf2::Vector3(centre[0] + std::cos(yaw) * stand::PUCK_L / 2, centre[1] + std::sin(yaw) * stand::PUCK_L / 2,
                                                 box_top_m_ + stand::PUCK_W / 2);
      const int tag = g.tag;
      steps->insert(steps->end(), {
        [this, tag, pc, pe](ArmIO::Cb c) { arm_.hand_target_end(tag, pe.x(), pe.y(), pe.z(), pc.x(), pc.y(), pc.z()); c(true); },
        ctx(0, true, true), arm_.wait_step(STAND_GRASP_HOLD_S),
        ctx(1, false, true),
        arm_.traj_step([run](const stand::Q5 & now) { return stand::sample_move(now, run->qs[1], STAND_GRASP_VS, TRAJ_DT); }),
        ctx(3, true, false)});
    } else {
      steps->push_back(arm_.traj_step([run, vs](const stand::Q5 & now) { return stand::sample_move(now, run->qs[1], vs, TRAJ_DT); }));
    }
    steps->insert(steps->end(), {
      arm_.grip_timed_step(pinch, close_a_, 0.0),
      arm_.grip_settle_step(grip_a_, empty_m_, grip_timeout_s_),
      arm_.grip_timed_step(hold_m, pinch_a_, STAND_PINCH_S)});  // 퍽이 손가락 사이에서 돌 만큼 낮춤 — 목표도 퍽 폭으로 풀어 누르지 않음
    if (learned) steps->push_back(ctx(4, false, false));
    const auto arc = arm_.traj_step([wp, sp, avs](const stand::Q5 &) { return stand::sample_arc(wp(), sp, avs, STAND_HANG_S, TRAJ_DT); });
    const double rel_deg = get_parameter("stand_release_deg").as_double();
    if (rel_deg > 0 && sp.drag_m <= 0 && rel_deg < sp.arc_end_deg) {
      // 호 도중 놓기: 호 경유점 wp[1 + i]가 i × arc_step_deg — 그 경유점에 가장 가까운 표본 시각에 벌림 (궤적은 그대로 끝까지)
      steps->push_back([this, arc, wp, sp, avs, rel_deg, open](ArmIO::Cb cb) {
        const auto q = wp();
        const auto smp = stand::sample_arc(q, sp, avs, STAND_HANG_S, TRAJ_DT);
        const auto & target = q[1 + static_cast<size_t>(std::ceil(rel_deg / sp.arc_step_deg))];
        size_t best = 0;
        double best_d = 1e9;
        for (size_t k = 0; k < smp.size(); ++k) {
          double dk = 0;
          for (int j = 0; j < 5; ++j) dk = std::max(dk, std::abs(smp[k][j] - target[j]));
          if (dk < best_d) { best_d = dk; best = k; }
        }
        const double t_rel = (best + 1) * TRAJ_DT;
        stage("호 " + std::to_string(static_cast<int>(rel_deg)) + "°에서 벌림 (" + std::to_string(t_rel).substr(0, 4) + " s)");
        if (release_timer_) release_timer_->cancel();
        release_timer_ = create_wall_timer(std::chrono::duration<double>(t_rel), [this, open]() {
          release_timer_->cancel();
          arm_.set_gripper(open, [](bool) {});
        });
        arc(cb);
      });
    } else {
      steps->push_back(arc);
    }
    if (learned) steps->push_back(ctx(5, true, false));
    steps->insert(steps->end(), {
      arm_.wait_step(STAND_SETTLE_S),
      arm_.grip_timed_step(open, -1.0, STAND_OPEN_S),
      arm_.traj_step([wp, vs](const stand::Q5 & now) { return stand::sample_move(now, wp().back(), vs, TRAJ_DT); })});
    if (learned) stage("stand_grasp 정책: 위·내려가기 학습 보정");
    arm_.run_steps(steps, 0, [this, run, done, learned](bool ok2) {
      if (learned) {  // 보정 끔 — Δ가 0으로 빠지며 팔이 조금 움직임: 다음 MoveIt 전에 멈출 때까지 (contact_step과 같음)
        arm_.ctx_off();
        arm_.hand_target("none", -1, -1);
        arm_.settle_next();
      }
      if (ok2) { done_qs_ = run->qs; done(true, ""); return; }
      const std::string why2 = !arm_.grip_fail().empty() ? arm_.grip_fail() : arm_.contact() ? "arm_contact" : "stand_failed";
      arm_.set_gripper(open_m_, [done, why2](bool) { done(false, why2); });  // 잡은 채 멈추지 않게 벌리고 실패
    });
  }

  // ---------- 쥘 끝 고르기 (11-08 실기: 상자 가장자리 쪽으로 누운 퍽은 세우다 상자 밖으로 떨어짐)
  // 선 퍽 자리 = 피벗(가까운 끝 바닥 모서리) − u·W/2. 그 자리가 상자 가장자리에 stand_end_margin_m보다 가까우면
  // 반대 끝(yaw + π)을 잡아 반대로 세우는 경로를 풀어 보고, 여유가 더 크고 풀리면 그쪽으로. 상자를 못 보면 그대로 (먼 끝)
  bool start_box(double cx, double cy, double & bx, double & by, double & byaw) {
    const auto m = objects_;
    if (!m) return false;
    tf2::Transform t;
    try {
      const auto tr = arm_.tf().lookupTransform("base_footprint", m->header.frame_id.empty() ? "map" : m->header.frame_id,
                                                tf2::TimePointZero).transform;
      t = tf2::Transform(tf2::Quaternion(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w),
                         tf2::Vector3(tr.translation.x, tr.translation.y, tr.translation.z));
    } catch (const std::exception &) {
      return false;
    }
    double best = 0.2;  // 퍽 가운데에서 이 안의 시작 상자만
    for (const auto & o : m->objects) {
      if (o.class_name != "start_box") continue;
      const auto & p = o.pose.pose.position;
      const auto & q = o.pose.pose.orientation;
      const tf2::Vector3 b = t * tf2::Vector3(p.x, p.y, p.z);
      const double d = std::hypot(b.x() - cx, b.y() - cy);
      if (d >= best) continue;
      best = d;
      bx = b.x();
      by = b.y();
      const tf2::Vector3 a = t.getBasis() * tf2::Vector3(1.0 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 0.0);
      byaw = std::atan2(a.y(), a.x());
    }
    return best < 0.2;
  }

  // 후보 = (먼 끝 / 반대 끝) × (손목 roll stand_flip_roll / 반대) 를 모두 풀어 보고 점수가 가장 높은 것을 고른다. 점수 =
  //   w_margin × 선 자리 여유 (상자 가장자리, stand_end_margin_m에서 포화 · 상자를 못 보면 0) +
  //   w_slack × 관절 한계 여유 (0.35 rad에서 포화 — 해가 넓은 쪽) + w_cam × (stand_flip_roll대로 = 카메라가 퍽 몸통 쪽) + w_far × 먼 끝
  // 풀리지 않는 후보는 뺀다. 고른 roll은 이번 실행의 sp_에 반영 (재계획이 같은 자세로 가게)
  // 먼 누운 퍽을 로봇 쪽으로 끌어옴: 로봇에 가까운 끝을 세우기와 같은 높이·방향으로 쥐고, 로봇 쪽(수평)으로 끈 뒤 놓고 위로
  void pull(const stand::V3 & centre, double yaw, bool is_box, double dist, Done done) {
    const double to = get_parameter("stand_pull_to_m").as_double();
    const double d = std::clamp(dist - to, 0.03, 0.10);
    stand::V3 u = {std::cos(yaw), std::sin(yaw), 0.0};
    const stand::V3 e1 = {centre[0] + u[0] * 0.03, centre[1] + u[1] * 0.03, 0.0}, e2 = {centre[0] - u[0] * 0.03, centre[1] - u[1] * 0.03, 0.0};
    const bool first_near = std::hypot(e1[0], e1[1]) < std::hypot(e2[0], e2[1]);
    if (!first_near) u = {-u[0], -u[1], 0.0};  // u = 가운데 → 로봇 쪽 끝
    const double inset = 0.008;  // 끝에서 손가락 반쯤 안쪽 (패드가 퍽 위에)
    const double h = (is_box ? sp_.contact_h_box : sp_.contact_h_cyl) + stand::GRASP_Z - stand::PUCK_H + stand::GRIP_DEPTH / 2;
    const stand::V3 g = {centre[0] + u[0] * (0.03 - inset), centre[1] + u[1] * (0.03 - inset), box_top_m_ + h};
    const double gr = std::hypot(g[0], g[1]);
    const stand::V3 dir = {-g[0] / gr, -g[1] / gr, 0.0};  // 로봇 쪽 수평
    const stand::V3 above = {g[0], g[1], g[2] + sp_.above_m}, pulled = {g[0] + dir[0] * d, g[1] + dir[1] * d, g[2] + 0.002};
    const stand::V3 up = {pulled[0], pulled[1], pulled[2] + sp_.lift_away_m};
    const double close_yaw = yaw + M_PI / 2 + (sp_.flip_roll ? M_PI : 0.0);
    std::vector<stand::Q5> qs;
    stand::Q5 near = {std::atan2(above[1], above[0]), -0.2, 0.7, 1.0, 0.0}, q;
    const stand::V3 pts[] = {above, g, pulled, up};
    for (int i = 0; i < 4; ++i) {
      bool ok = stand::ik_down_sym(pts[i], close_yaw, near, q);
      // 손 빼기 높이(stand_lift_away_m 0.13)는 먼 자리에서 손을 아래로 향한 채 못 감 — 세우기처럼 6 → 4 → 3 cm로 낮춤
      //   (11-15 실기: 0.287 m 퍽에서 위·쥐기·끌기는 풀리고 위로만 실패)
      if (!ok && i == 3)
        for (double hh : {0.06, 0.04, 0.03})
          if ((ok = stand::ik_down_sym({up[0], up[1], pulled[2] + hh}, close_yaw, near, q))) break;
      if (!ok) { done(false, "stand_pull_unreachable"); return; }
      qs.push_back(q);
      near = q;
    }
    stage("먼 퍽 끌어오기: 로봇에서 " + std::to_string(static_cast<int>(dist * 1000)) + " mm → " + std::to_string(static_cast<int>(d * 1000)) +
          " mm 당김 (세우기는 다음 stand)");
    arm_.clear_flags();
    const double vs = vs_, pvs = get_parameter("stand_pull_vel_scale").as_double();
    const auto to_q = [](const stand::Q5 & a, double v) {
      return [a, v](const stand::Q5 & now) { return stand::sample_move(now, a, v, TRAJ_DT); };
    };
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(std::vector<ArmIO::Step>{
      arm_.gripper_step(open_m_),
      arm_.joints_step(std::vector<double>(qs[0].begin(), qs[0].end())),  // 위 (MoveIt — 충돌 확인)
      arm_.wait_step(hold_s_),
      arm_.traj_step(to_q(qs[1], vs)),                                    // 내려감
      arm_.grip_timed_step(pinch_m_, close_a_, 0.0),
      arm_.grip_settle_step(grip_a_, empty_m_, grip_timeout_s_),          // 쥠 (stand와 같은 판정)
      arm_.traj_step(to_q(qs[2], pvs)),                                   // 로봇 쪽으로 끎
      arm_.grip_timed_step(open_m_, -1.0, STAND_OPEN_S),
      arm_.traj_step(to_q(qs[3], vs))});                                  // 위로
    arm_.run_steps(steps, 0, [this, done](bool ok) {
      if (ok) { done(true, ""); return; }
      const std::string why = !arm_.grip_fail().empty() ? arm_.grip_fail() : arm_.contact() ? "arm_contact" : "stand_pull_failed";
      arm_.set_gripper(open_m_, [done, why](bool) { done(false, why); });
    });
  }

  void choose_end(const stand::V3 & centre, bool is_box, bool /*ok*/, double & yaw, std::vector<stand::Q5> & qs) {
    const double need = get_parameter("stand_end_margin_m").as_double();
    const double w_m = get_parameter("stand_score_margin").as_double(), w_s = get_parameter("stand_score_slack").as_double();
    const double w_c = get_parameter("stand_score_cam").as_double(), w_f = get_parameter("stand_score_far").as_double();
    double bx = 0, by = 0, byaw = 0;
    const bool box = need > 0.0 && start_box(centre[0], centre[1], bx, by, byaw);
    constexpr double BOX_HALF = 0.205 / 2;
    const auto margin = [&](double yw) {  // 선 퍽 가장자리 ~ 상자 가장자리 [m] (상자 좌표에서, 퍽 반폭만큼 안쪽)
      const double sx = centre[0] - std::cos(yw) * (stand::PUCK_L + stand::PUCK_W) / 2 - bx;
      const double sy = centre[1] - std::sin(yw) * (stand::PUCK_L + stand::PUCK_W) / 2 - by;
      const double c = std::cos(byaw), s = std::sin(byaw);
      return BOX_HALF - stand::PUCK_W / 2 - std::max(std::abs(c * sx + s * sy), std::abs(-s * sx + c * sy));
    };
    const auto sat = [](double v, double lo, double hi) { return std::min(std::max(v, lo), hi); };
    const double yaws[2] = {yaw, std::remainder(yaw + M_PI, 2 * M_PI)};
    double best = -1e9, best_m = 0.0;
    int best_e = -1, best_r = -1;
    std::vector<stand::Q5> best_qs;
    for (int e = 0; e < 2; ++e) {
      const double m = box ? margin(yaws[e]) : 0.0;
      for (int r = 0; r < 2; ++r) {
        stand::Params sp = sp_;
        sp.flip_roll = r == 0 ? sp_.flip_roll : !sp_.flip_roll;
        std::vector<stand::Q5> q;
        std::string w;
        if (!stand::plan_ik(centre, yaws[e], is_box, sp, q, w)) continue;
        const double sc = w_m * (box ? sat(m / need, -1.0, 1.0) : 0.0) + w_s * sat(stand::limit_slack(q) / 0.35, 0.0, 1.0) +
                          w_c * (r == 0) + w_f * (e == 0);
        if (sc > best) { best = sc; best_e = e; best_r = r; best_qs = q; best_m = m; }
      }
    }
    if (best_e < 0) return;  // 어느 후보도 안 풀림 — 호출한 쪽의 why로 실패
    yaw = yaws[best_e];
    qs = best_qs;
    sp_.flip_roll = best_r == 0 ? sp_.flip_roll : !sp_.flip_roll;
    char buf[160];
    std::snprintf(buf, sizeof buf, "쥘 끝: %s · 손목 roll %s · 점수 %.2f (선 자리 여유 %s%.0f mm)", best_e == 0 ? "먼 끝" : "반대 끝",
                  sp_.flip_roll ? "180°" : "0°", best, box ? "" : "모름 ", best_m * 1000);
    stage(buf);
  }

  // ---------- 호 방식: 내려가기 전 손 카메라로 다시 재기
  struct SRun {
    std::vector<stand::Q5> qs;
    stand::V3 centre0{};
    double yaw0 = 0;
    bool is_box = true;
  };

  // 잴 자세: 쥐는 자리에서 로봇 쪽으로 stand_cam_back_m, 위로 stand_cam_above_m (못 닿으면 1 cm씩 낮춤, 4 cm 아래면 안 잼)
  bool look_pose(const stand::V3 & centre, double yaw, bool is_box, const stand::Q5 & near, stand::Q5 & out) {
    double close_yaw;
    const auto pts = stand::waypoints(centre, yaw, is_box, sp_, close_yaw);
    const double back = get_parameter("stand_cam_back_m").as_double();
    for (double h = get_parameter("stand_cam_above_m").as_double(); h > 0.04 - 1e-9; h -= 0.01) {
      const stand::V3 p = {pts[1][0] - back * std::cos(yaw), pts[1][1] - back * std::sin(yaw), pts[1][2] + h};
      if (stand::ik_down(p, close_yaw, near, out)) return true;
    }
    stage("손 카메라: 잴 자세에 못 닿음 — inspect 값 그대로");
    return false;
  }

  ArmIO::Step cam_step(std::shared_ptr<SRun> run, int tag, double gx, double gy, double gz) {
    return [this, run, tag, gx, gy, gz](ArmIO::Cb next) {
      arm_.hand_target("puck", tag, -1, gx, gy, gz);
      struct Acc { std::vector<double> x, y, dyaw; int64_t last = 0; std::chrono::steady_clock::time_point t0; };
      auto acc = std::make_shared<Acc>();
      acc->t0 = std::chrono::steady_clock::now();
      const double wait_s = get_parameter("stand_cam_wait_s").as_double();
      cam_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this, run, acc, wait_s, next]() {
        const auto m = hr_;
        stand::Q5 q;
        if (m && m->valid && m->age < 0.45 && arm_.arm_now(q)) {
          const int64_t t = rclcpp::Time(m->header.stamp).nanoseconds();
          if (t != acc->last) {  // hand_relative는 같은 검출을 새 검출이 올 때까지 다시 보냄 — 영상 시각이 바뀐 것만
            acc->last = t;
            const auto e = stand::fk(q);  // rel은 base_link 기준 차이 — base_footprint와 방향이 같아 그대로 더함
            acc->x.push_back(e[0] + m->rel.x);
            acc->y.push_back(e[1] + m->rel.y);
            // 손 카메라가 축을 못 재면 axis = (0, 0, 1) (실기 11/08: 누운 퍽도 늘 그랬음) — 위치만 쓰고 축은 inspect 값
            if (std::abs(m->axis.z) < 0.5)
              acc->dyaw.push_back(std::remainder(stand::away_yaw(std::atan2(m->axis.y, m->axis.x)) - run->yaw0, M_PI));  // 축은 180° 대칭
          }
        }
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - acc->t0).count() < wait_s) return;
        cam_timer_->cancel();
        arm_.hand_target("none", -1, -1);
        cam_fix(*run, acc->x, acc->y, acc->dyaw);
        next(true);
      });
    };
  }

  static double median(std::vector<double> v) {
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  }

  // 잰 값(중앙값)으로 경유점을 다시 풂 — 못 보거나 못 풀면 inspect 값 그대로
  void cam_fix(SRun & run, const std::vector<double> & xs, const std::vector<double> & ys, const std::vector<double> & dyaws) {
    const int n = static_cast<int>(xs.size());
    if (n < get_parameter("stand_cam_min_n").as_int()) {
      stage("손 카메라: 퍽을 " + std::to_string(n) + "번만 봄 — inspect 값 그대로");
      return;
    }
    const double max_m = get_parameter("stand_cam_max_m").as_double();
    const double max_a = get_parameter("stand_cam_max_deg").as_double() * M_PI / 180;
    double dx = median(xs) - run.centre0[0], dy = median(ys) - run.centre0[1];
    const double d = std::hypot(dx, dy);
    if (d > max_m) { dx *= max_m / d; dy *= max_m / d; }
    const double dyaw = dyaws.size() >= xs.size() / 2 + 1 ? std::clamp(median(dyaws), -max_a, max_a) : 0.0;  // 축은 반 넘게 잰 때만
    const stand::V3 c = {run.centre0[0] + dx, run.centre0[1] + dy, run.centre0[2]};
    std::vector<stand::Q5> qs;
    std::string why;
    if (!stand::plan_ik(c, run.yaw0 + dyaw, run.is_box, sp_, qs, why)) {
      stage("손 카메라: 고친 자리 " + why + " — inspect 값 그대로");
      return;
    }
    run.qs = qs;
    char buf[160];
    std::snprintf(buf, sizeof buf, "손 카메라 %d번: 퍽 (%+.1f, %+.1f) mm · 축 %+.1f°%s 고침%s", n, dx * 1000, dy * 1000,
                  dyaw * 180 / M_PI, dyaws.empty() ? " (축 못 잼)" : "", d > max_m ? " (한도에서 자름)" : "");
    stage(buf);
  }

  void stand_pick(Done done) {
    if (done_qs_.size() < 2) { done(false, "stand_pick_no_stand"); return; }
    const stand::Q5 grasp = done_qs_[done_qs_.size() - 2], lift = done_qs_.back();
    const double vs = vs_;
    arm_.clear_flags();
    stage("선 퍽 다시 쥐기");
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(std::vector<ArmIO::Step>{
      arm_.traj_step([lift, vs](const stand::Q5 & now) { return stand::sample_move(now, lift, vs, TRAJ_DT); }),   // 선 퍽 위
      arm_.traj_step([grasp, vs](const stand::Q5 & now) { return stand::sample_move(now, grasp, vs, TRAJ_DT); }),  // 바로 아래로
      arm_.wait_step(hold_s_),                                                                                // 처짐 보정 쌓임
      arm_.grip_timed_step(arm_.grip_close_m(), arm_.grip_current_a(), REGRASP_CLOSE_S), arm_.held_step(),
      arm_.traj_step([lift, vs](const stand::Q5 & now) { return stand::sample_move(now, lift, vs, TRAJ_DT); }),
      arm_.held_step()});  // 들어 올린 뒤 한 번 더 — 모서리만 물었다 놓치면 여기서 빈손 (11-15: 닫을 때 0.0048 m · 0.09 A로 '쥠' 뒤 놓침)
    arm_.run_steps(steps, 0, [this, done](bool ok) {
      if (!ok) {
        if (arm_.grasp_empty()) arm_.set_gripper(open_m_, [](bool) {});
        done(false, arm_.grasp_empty() ? "grasp_empty" : "stand_pick_failed");
        return;
      }
      done_qs_.clear();
      done(true, "");
    });
  }

  void refresh_params() {
    const auto d = [this](const char * n) { return get_parameter(n).as_double(); };
    open_m_ = d("stand_open_m"); pinch_m_ = d("stand_pinch_m");
    close_a_ = d("stand_close_current_a"); pinch_a_ = d("stand_pinch_current_a"); grip_a_ = d("stand_grip_current_a"); empty_m_ = d("stand_empty_m");
    sp_.drag_m = d("stand_drag_m"); sp_.drag_lift_m = d("stand_drag_lift_m"); sp_.arc_end_deg = d("stand_arc_end_deg");
    sp_.set_dz_m = d("stand_set_dz_m"); sp_.tilt_over_m = d("stand_tilt_over_m"); sp_.arc_return = get_parameter("stand_arc_return").as_bool(); sp_.flip_roll = get_parameter("stand_flip_roll").as_bool();
    sp_.above_m = d("stand_above_m"); sp_.lift_away_m = d("stand_lift_away_m"); sp_.arc_k = d("stand_arc_k");
    sp_.contact_h_box = d("stand_contact_h_box_m"); sp_.contact_h_cyl = d("stand_contact_h_cyl_m");
    sp_.pinch_from_end_m = d("stand_pinch_from_end_m");
    vs_ = d("stand_vel_scale"); arc_vs_ = d("stand_arc_vel_scale"); hold_s_ = d("stand_hold_s");
  }

  ArmIO arm_;
  SkillModel policy_, grasp_model_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr cam_timer_, release_timer_;
  rclcpp::Subscription<perception::msg::HandRelative>::SharedPtr hr_sub_;
  perception::msg::HandRelative::ConstSharedPtr hr_;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr objects_sub_;
  perception::msg::TrackedObjectArray::ConstSharedPtr objects_;
  std::string waypoints_ = "ik";
  double box_top_m_ = 0.07, vs_ = 0.3, arc_vs_ = 0.5;
  double open_m_ = 0.012, pinch_m_ = 0.007, close_a_ = 0.01614, pinch_a_ = 0.00538, grip_a_ = 0.007, empty_m_ = 0.003, grip_timeout_s_ = 4.0, hold_s_ = 0.3;
  stand::Params sp_;
  stand::Table table_;
  std::vector<stand::Q5> done_qs_;  // 마지막 stand 경유점 — stand_pick이 같은 손목으로 다시 잡음
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::StandSkill)
