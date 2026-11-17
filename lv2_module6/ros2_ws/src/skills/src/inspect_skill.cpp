// skill_inspect — /skills/inspect: gripper_cam으로 보기 (학습 정책 없음). 읽었는지(details tag)는 coordinator · mission_manager도 /tracked_objects로 봄
//   점 보기 (target 비움): goal.pose(map) 점 위 inspect_height_m에 손끝을 두고 inspect_wait_s 기다림 (camera_look_down).
//     퍽 윗면 16mm AprilTag는 gripper_cam이 위에서 봐야 읽힘. values[0] = 손 방향 yaw (로봇 → 점) — 없으면 자유
//   상자 보기 (target "box"): goal.pose = 상자 위 퍽 윗면 높이의 상자 가운데 (map), values[0] = 기대 퍽 수 (없으면 2)
//     ⓪ 지금 자세에서 이미 기대 수만큼 보이고 태그·자세도 다 알면 끝 (팔 안 움직임)
//     ① pose_look 관절 자세(arm.yaml — skill_pose "look"과 같은 값)로 → inspect_look_wait_s 기다림
//     ② 상자 가운데에서 inspect_box_radius_m 안 퍽 트랙이 기대 수 이상이고 모두 태그·자세를 알면 끝
//     ③ 아니면 본 퍽(모자라거나 태그·자세 모르는 퍽이 있으면 상자 가운데도)을 한 장에 담는 자세(look_plan)로 → 기다림 → 다시 셈. 모자라면 inspect_partial
//     11/07 Isaac 전체 스택 퍽 40개: 1차 자세만 85 % · 다시 담으면 92 % (isaac/scripts/demo/inspect_test.py center)
#include <cmath>
#include <limits>
#include <mutex>

#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/look_plan.hpp"
#include "skills/skill_node.hpp"
#include "tf2/LinearMath/Transform.hpp"

namespace skills {

class InspectSkill : public SkillNode {
public:
  explicit InspectSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_inspect", options), arm_(*this) {
    height_m_ = declare_parameter<double>("inspect_height_m", 0.15);  // [m] 손끝을 퍽 윗면 위 이만큼 (16mm 태그 ≈ 50px)
    wait_s_ = declare_parameter<double>("inspect_wait_s", 1.0);       // [s] perception 추론 4Hz × 몇 장
    look_q_ = declare_parameter<std::vector<double>>("pose_look", {0.0, -1.682497, 0.324631, 1.926843, 0.0});  // arm.yaml이 덮음
    look_wait_s_ = declare_parameter<double>("inspect_look_wait_s", 1.5);  // [s] 상자 보기 자세마다 (궤적 끝 · 처짐 보정 수렴 + 추론)
    box_radius_m_ = declare_parameter<double>("inspect_box_radius_m", 0.16);  // [m] 상자 가운데에서 이 안 퍽만 셈 (반 변 0.1025 + 여유)
    // 퍽이 몸 가까이 있어 look 자세 · 위에서 보기로 다 못 보면 look 자세로 멈추지 않고 back_speed로 back_max_m까지 물러나며 봄 —
    //   다 보이면 물러난 만큼 앞으로 되돌아옴 (그다음 pick이 원래 도킹 자리에서) · 0이면 끔 · 멈추지 않고 후진 (11-16 사용자) (11-16 시뮬: 하나 남은 퍽이 가까워 inspect_failed)
    back_speed_ = declare_parameter<double>("inspect_back_speed", 0.03);  // [m/s] 멈추지 않고 이 속도로 물러나며 0.25 s마다 셈
    back_max_m_ = declare_parameter<double>("inspect_back_max_m", 0.20);
    backup_ = rclcpp_action::create_client<nav2_msgs::action::BackUp>(this, "/backup");
    drive_ = rclcpp_action::create_client<nav2_msgs::action::DriveOnHeading>(this, "/drive_on_heading");
    objects_sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
      [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { std::lock_guard<std::mutex> lk(mu_); objects_ = m; });
    serve("inspect", [this](const SkillGoal & g, Done done) {
      when_ready([this]() { return arm_.moveit_ready(); }, "arm_unavailable", done, [this, g, done]() {
        if (g.target == "box") look_box(g, done);
        else look_point(g, done);
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  void look_point(const SkillGoal & g, Done done) {
    auto at = g.pose;
    if (at.header.frame_id.empty()) at.header.frame_id = "map";
    at.header.stamp = now();
    at.pose.position.z += height_m_;
    at.pose.orientation = geometry_msgs::msg::Quaternion();
    const double yaw = g.values.empty() ? std::numeric_limits<double>::quiet_NaN() : g.values[0];
    stage("위에서 내려다보기");
    const auto look = [this, done, g](bool ok) {
      if (!ok) { look_back(g.pose, 1, done); return; }  // 위에서 못 봄 (가까워 IK 밖 등) — look 자세로 물러나며
      stage("기다림");
      arm_.wait_step(wait_s_)([done](bool) { done(true, ""); });
    };
    // 손을 아래로 향한 채 높이 못 가는 자리가 많음 — 해석 IK로 낮춰 가며 (시뮬 camera_look_down 0.08). 다 안 되면 MoveIt
    ArmIO::Q5 now, q;
    if (arm_.arm_now(now))
      for (const double drop : {0.0, height_m_ - 0.10, height_m_ - 0.08}) {
        auto p = at;
        p.pose.position.z -= drop;
        if (arm_.ik_ee(p, now, q)) { arm_.move_joints(std::vector<double>(q.begin(), q.end()), look); return; }
      }
    arm_.move_arm(at, look, yaw);
  }

  // c 좌표계(보통 map) → base_footprint
  bool to_base(const geometry_msgs::msg::PoseStamped & c, tf2::Transform & t) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = arm_.tf().lookupTransform("base_footprint", c.header.frame_id.empty() ? "map" : c.header.frame_id, tf2::TimePointZero);
    } catch (const std::exception &) {
      return false;
    }
    const auto & r = tf.transform.rotation;
    const auto & tr = tf.transform.translation;
    t = tf2::Transform(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(tr.x, tr.y, tr.z));
    return true;
  }

  // 상자 가운데 c에서 box_radius_m_ 안 퍽 트랙 → base_footprint 윗면 점들 (z = c.z) · 다 본 수 (태그와 자세를 앎)
  //   트랙 중복은 perception이 합침 (merge_tracks · merge_overlap_dets)
  bool box_pucks(const geometry_msgs::msg::PoseStamped & c, std::vector<look::V3> & tops, int & tagged) {
    tops.clear();
    tagged = 0;
    tf2::Transform t;
    if (!to_base(c, t)) return false;
    std::lock_guard<std::mutex> lk(mu_);
    if (!objects_) return true;
    for (const auto & o : objects_->objects) {
      if (o.class_name.rfind("puck", 0) != 0) continue;
      const auto & p = o.pose.pose.position;
      if (std::hypot(p.x - c.pose.position.x, p.y - c.pose.position.y) > box_radius_m_) continue;
      const tf2::Vector3 b = t * tf2::Vector3(p.x, p.y, c.pose.position.z);
      tops.push_back({b.x(), b.y(), b.z()});
      // 다 봤음 = 태그 + 자세(standing · lying)까지 — 자세를 모르면('?') coordinator가 세울지 집을지 못 정함 (11/09 Isaac)
      bool tag = false, posture = false;
      for (const auto & kv : o.details) {
        if (kv.key == "tag" && !kv.value.empty()) tag = true;
        if (kv.key == "posture" && !kv.value.empty()) posture = true;
      }
      if (tag && posture) ++tagged;
    }
    return true;
  }

  // look 자세로 멈추지 않고 천천히 뒤로 물러나며 c 둘레 퍽을 want개 · 태그까지 다 볼 때까지 (back_max까지) — 보이는 순간 멈추고
  //   한 번 더 확인한 뒤 물러난 만큼 앞으로 되돌아옴
  bool all_seen(const geometry_msgs::msg::PoseStamped & c, size_t want) {
    std::vector<look::V3> tops;
    int tagged = 0;
    return box_pucks(c, tops, tagged) && tops.size() >= want && tagged >= static_cast<int>(tops.size());
  }

  void look_back(const geometry_msgs::msg::PoseStamped & c0, size_t want, Done done) {
    auto c = c0;
    if (c.header.frame_id.empty()) c.header.frame_id = "map";
    if (back_speed_ <= 0.0 || back_max_m_ <= 0.0 || look_q_.size() != 5 || !backup_->action_server_is_ready()) {
      done(false, "inspect_partial");
      return;
    }
    stage("look 자세로 물러나며 보기");
    arm_.move_joints(look_q_, [this, c, want, done](bool ok) {
      if (!ok) { done(false, "inspect_failed"); return; }
      arm_.wait_step(look_wait_s_)([this, c, want, done](bool) {
        if (all_seen(c, want)) { done(true, ""); return; }
        auto backed = std::make_shared<double>(0.0);
        auto stopped = std::make_shared<bool>(false);  // 다 보여서 멈춤 (취소)
        auto fin = std::make_shared<bool>(false);
        // 끝 (다 보여 취소 · 끝까지 감 · 막힘): 멈춘 자리에서 한 번 더 보고 → 제자리로
        auto finish = [this, c, want, done, backed, fin](const char * why) {
          if (*fin) return;
          *fin = true;
          if (back_timer_) back_timer_->cancel();
          arm_.wait_step(0.5)([this, c, want, done, backed, why](bool) {
            const bool all = all_seen(c, want);
            RCLCPP_INFO(get_logger(), "물러나 보기 %.2f m (%s) — %s", *backed, why, all ? "다 보임" : "그래도 모자람");
            come_back(*backed, [done, all]() { done(all, all ? "" : "inspect_partial"); });
          });
        };
        stage("뒤로 (보이면 멈춤)");
        nav2_msgs::action::BackUp::Goal g;
        g.target.x = back_max_m_;
        g.speed = static_cast<float>(back_speed_);
        g.time_allowance.sec = static_cast<int32_t>(back_max_m_ / back_speed_ + 4);
        g.disable_collision_checks = false;  // 뒤는 카메라가 못 봄 — costmap 검사로 벽 앞에서 멈춤
        rclcpp_action::Client<nav2_msgs::action::BackUp>::SendGoalOptions o;
        o.feedback_callback = [backed](rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::SharedPtr,
                                       const std::shared_ptr<const nav2_msgs::action::BackUp::Feedback> f) {
          *backed = std::abs(f->distance_traveled);
        };
        o.result_callback = [stopped, finish](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::WrappedResult & r) {
          finish(*stopped ? "보여서 멈춤" : r.code == rclcpp_action::ResultCode::SUCCEEDED ? "끝까지" : "막힘");
        };
        o.goal_response_callback = [this, c, want, stopped, finish](
            const rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::SharedPtr & h) {
          if (!h) { finish("거절"); return; }
          back_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this, c, want, stopped, h]() {
            if (*stopped || !all_seen(c, want)) return;
            *stopped = true;
            backup_->async_cancel_goal(h);  // result_callback(취소)이 finish
          });
        };
        backup_->async_send_goal(g, o);
      });
    });
  }

  // 물러난 만큼 곧게 앞으로 (도킹 자리로) — 실패해도 then
  void come_back(double d, std::function<void()> then) {
    if (d <= 1e-3 || !drive_->action_server_is_ready()) { then(); return; }
    stage("제자리로");
    nav2_msgs::action::DriveOnHeading::Goal g;
    g.target.x = d;
    g.speed = 0.05f;
    g.time_allowance.sec = static_cast<int32_t>(d / 0.05 + 4);
    g.disable_collision_checks = true;  // 방금 지나온 길 — 앞의 상자는 costmap 팽창 안이라 검사하면 거절됨
    rclcpp_action::Client<nav2_msgs::action::DriveOnHeading>::SendGoalOptions o;
    o.result_callback = [then](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::DriveOnHeading>::WrappedResult &) { then(); };
    o.goal_response_callback = [then](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::DriveOnHeading>::SharedPtr & h) { if (!h) then(); };
    drive_->async_send_goal(g, o);
  }

  void look_box(const SkillGoal & g, Done done) {
    const auto c = g.pose;
    const size_t want = g.values.empty() ? 2 : static_cast<size_t>(std::max(1.0, g.values[0]));
    if (look_q_.size() != 5) { done(false, "inspect_no_look_pose"); return; }
    {  // 지금 자세(보통 stow — gripper_cam이 상자 쪽)에서 이미 퍽이 다 보이고 태그도 읽었으면 팔을 안 움직임
      //   11/07 Isaac 6배치: stow에서 둘 다 태그까지 1/6 (태그 중심이라 위치 1 mm 안) · 태그 없이 본 퍽은 19~39 mm 틀려 이 조건으로만
      std::vector<look::V3> tops;
      int tagged = 0;
      if (box_pucks(c, tops, tagged) && tops.size() >= want && tagged >= static_cast<int>(tops.size())) {
        stage("이미 보임 — 자세 그대로");
        done(true, "");
        return;
      }
    }
    stage("look 자세");
    arm_.move_joints(look_q_, [this, c, want, done](bool ok) {
      if (!ok) { done(false, "inspect_failed"); return; }
      arm_.wait_step(look_wait_s_)([this, c, want, done](bool) {
        std::vector<look::V3> tops;
        int tagged = 0;
        if (!box_pucks(c, tops, tagged)) { done(false, "inspect_no_tf"); return; }
        if (tops.size() >= want && tagged >= static_cast<int>(tops.size())) { done(true, ""); return; }
        // 다시 담기: 본 퍽 + (모자라거나 태그 못 읽은 퍽이 있으면) 상자 가운데 — 못 본 퍽이 상자 위 어딘가에 있음
        //   태그 못 읽은 퍽 위치는 옆면 depth라 19~39 mm 틀림 — 그 점만 담으면 퍽을 놓친 자세가 됨 (11/09 Isaac: 살짝 보인 퍽)
        tf2::Transform t;
        if ((tops.size() < want || tagged < static_cast<int>(tops.size())) && to_base(c, t)) {
          const tf2::Vector3 b = t * tf2::Vector3(c.pose.position.x, c.pose.position.y, c.pose.position.z);
          tops.push_back({b.x(), b.y(), b.z()});
        }
        look::Q5 q;
        look::Score s;
        if (!look::search(tops, look::Params{}, q, s)) { done(false, "inspect_partial"); return; }
        stage("다시 담기 (점 " + std::to_string(tops.size()) + "개 · 덮음 " + std::to_string(static_cast<int>(100 * s.cover)) + "%)");
        arm_.move_joints(std::vector<double>(q.begin(), q.end()), [this, c, want, done](bool ok2) {
          if (!ok2) { done(false, "inspect_failed"); return; }
          arm_.wait_step(look_wait_s_)([this, c, want, done](bool) {
            std::vector<look::V3> tops2;
            int tagged2 = 0;
            box_pucks(c, tops2, tagged2);
            const bool all = tops2.size() >= want && tagged2 >= static_cast<int>(tops2.size());
            if (all) { done(true, ""); return; }
            look_back(c, want, done);  // 그래도 모자라면 look 자세로 물러나며
          });
        });
      });
    });
  }

  ArmIO arm_;
  double height_m_, wait_s_, look_wait_s_, box_radius_m_, back_speed_ = 0.03, back_max_m_ = 0.20;
  rclcpp::TimerBase::SharedPtr back_timer_;
  rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr backup_;
  rclcpp_action::Client<nav2_msgs::action::DriveOnHeading>::SharedPtr drive_;
  std::vector<double> look_q_;
  std::mutex mu_;
  perception::msg::TrackedObjectArray::ConstSharedPtr objects_;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr objects_sub_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::InspectSkill)
