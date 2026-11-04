// plan 기본 테스트: coordinator는 허용 동작(navigate·pick·place·inspect·say·pose·load·unload)이 아닌 계획을 거부하고 FAILED를 보낸다 (규칙 1)
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "plan/coordinator.hpp"
#include "plan/vlm_task_planner.hpp"

using namespace plan;  // 노드 클래스·메시지 별칭 (include/plan/*.hpp)

// 조건이 맞을 때까지 step()으로 입력을 넣으며 spin. discovery 전에 보낸 메시지는 사라질 수 있어서 반복해 넣는다
template <class Step, class Done>
bool spin_until(rclcpp::Executor & exec, Step step, Done done, double timeout_s = 5.0) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    step();
    exec.spin_some(std::chrono::milliseconds(50));
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

class RosTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(RosTest, RejectsUnknownSkill) {
  auto node = std::make_shared<Coordinator>();
  auto probe = std::make_shared<rclcpp::Node>("probe");
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  auto objects_pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto goals_pub = probe->create_publisher<SubGoalArray>("/task/sub_goals", latched);
  TaskStatus::SharedPtr got;
  auto sub = probe->create_subscription<TaskStatus>("/task/status", latched,
    [&](TaskStatus::SharedPtr m) { got = m; });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  SubGoalArray plan;
  plan.plan_id = 7;
  SubGoal g;
  g.step = 1;
  g.skill = "fly";
  g.target = "3";
  plan.goals.push_back(g);
  goals_pub->publish(plan);
  ASSERT_TRUE(spin_until(exec,
    [&]() {
      TrackedObjectArray objects;
      perception::msg::TrackedObject puck;
      puck.id = 3;
      puck.class_name = "puck";
      objects.objects.push_back(puck);
      objects_pub->publish(objects);
    },
    [&]() { return got != nullptr; }));
  EXPECT_EQ(got->plan_id, 7u);
  EXPECT_EQ(got->state, TaskStatus::FAILED);
  EXPECT_EQ(got->reason.rfind("invalid_plan", 0), 0u) << got->reason;
}

// pose는 아는 자세 이름만 · load·unload의 target(칸 번호)은 물체 id로 검증하지 않음 (05-arm.md)
TEST_F(RosTest, ValidatesPoseAndSlotTargets) {
  auto node = std::make_shared<Coordinator>();
  auto probe = std::make_shared<rclcpp::Node>("probe_skills");
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  auto objects_pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto goals_pub = probe->create_publisher<SubGoalArray>("/task/sub_goals", latched);
  std::vector<TaskStatus> got;
  auto sub = probe->create_subscription<TaskStatus>("/task/status", latched,
    [&](TaskStatus::SharedPtr m) { got.push_back(*m); });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  auto send = [&](uint32_t id, const std::string & skill, const std::string & target) {
    SubGoalArray plan;
    plan.plan_id = id;
    SubGoal g;
    g.step = 1;
    g.skill = skill;
    g.target = target;
    plan.goals.push_back(g);
    goals_pub->publish(plan);
  };
  auto has = [&](uint32_t id, auto pred) { for (auto & s : got) if (s.plan_id == id && pred(s)) return true; return false; };
  send(21, "pose", "fly");
  ASSERT_TRUE(spin_until(exec, [&]() { objects_pub->publish(TrackedObjectArray()); },
    [&]() { return has(21, [](const TaskStatus & s) { return s.state == TaskStatus::FAILED; }); }));
  EXPECT_TRUE(has(21, [](const TaskStatus & s) { return s.reason == "invalid_plan:pose fly"; }));
  // 칸 5 — /tracked_objects에 id 5가 없어도 계획은 받음 (실행은 MoveIt 대기)
  send(22, "load", "5");
  ASSERT_TRUE(spin_until(exec, [&]() { objects_pub->publish(TrackedObjectArray()); },
    [&]() { return has(22, [](const TaskStatus & s) { return s.state == TaskStatus::RUNNING; }); }));
  EXPECT_FALSE(has(22, [](const TaskStatus & s) { return s.reason.rfind("invalid_plan", 0) == 0; }));
}

// 수동 모드로 바뀌면 진행 중 작업을 멈추고 FAILED(manual_override)를 보낸다
TEST_F(RosTest, ManualModeStopsRunningPlan) {
  auto node = std::make_shared<Coordinator>();
  auto probe = std::make_shared<rclcpp::Node>("probe2");
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  auto objects_pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto goals_pub = probe->create_publisher<SubGoalArray>("/task/sub_goals", latched);
  auto mode_pub = probe->create_publisher<std_msgs::msg::String>("/state/operation_mode", rclcpp::QoS(1).reliable().transient_local());
  std::vector<TaskStatus> got;
  auto sub = probe->create_subscription<TaskStatus>("/task/status", latched,
    [&](TaskStatus::SharedPtr m) { got.push_back(*m); });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  SubGoalArray plan;
  plan.plan_id = 8;
  SubGoal g;
  g.step = 1;
  g.skill = "navigate";
  g.target = "3";
  plan.goals.push_back(g);
  goals_pub->publish(plan);
  TrackedObjectArray objects;
  perception::msg::TrackedObject puck;
  puck.id = 3;
  puck.class_name = "puck";
  objects.objects.push_back(puck);
  // 계획 수락 (Nav2 서버가 없어 이동 단계에서 기다리는 중 = 실행 중)
  ASSERT_TRUE(spin_until(exec, [&]() { objects_pub->publish(objects); },
    [&]() { for (auto & s : got) if (s.plan_id == 8 && s.state == TaskStatus::RUNNING) return true; return false; }));
  std_msgs::msg::String manual;
  manual.data = "MANUAL";
  mode_pub->publish(manual);
  ASSERT_TRUE(spin_until(exec, [] {},
    [&]() { for (auto & s : got) if (s.plan_id == 8 && s.reason == "manual_override") return true; return false; }));
}

// 적재함 (MISSION ③): 칸 파라미터 해석 · 같은 곳 여러 개 비켜 놓기 · MOVE_TO_B 템플릿 = 싣고(place tray) → 이동 → 꺼내서(pick tray) 내려놓기
TEST(Tray, BasketSlot) {
  // 적재함 12칸 고르기 (load next · unload last · 칸 번호, 옛 "tray" = 같은 뜻)
  std::vector<std::string> filled(12, "");
  std::vector<int> order;
  std::string why;
  EXPECT_EQ(basket_slot("next", filled, order, true, why), 0);
  EXPECT_EQ(basket_slot("tray", filled, order, true, why), 0);
  EXPECT_EQ(basket_slot("last", filled, order, false, why), -1);
  EXPECT_EQ(why, "slot_empty");
  filled[0] = "box";
  filled[3] = "cyl";
  order = {3, 0};
  EXPECT_EQ(basket_slot("next", filled, order, true, why), 1);   // 빈 칸 중 가장 낮은 번호
  EXPECT_EQ(basket_slot("last", filled, order, false, why), 0);  // 마지막에 실은 칸
  EXPECT_EQ(basket_slot("tray", filled, order, false, why), 0);
  EXPECT_EQ(basket_slot("3", filled, order, false, why), 3);
  EXPECT_EQ(basket_slot("3", filled, order, true, why), -1);
  EXPECT_EQ(why, "slot_full");
  EXPECT_EQ(basket_slot("5", filled, order, false, why), -1);
  EXPECT_EQ(why, "slot_empty");
  EXPECT_EQ(basket_slot("12", filled, order, true, why), -1);
  EXPECT_EQ(why, "slot_invalid");
  EXPECT_EQ(basket_slot("-1", filled, order, true, why), -1);
  EXPECT_EQ(why, "slot_invalid");
  // load next는 쓰는 칸(usable)만 — 1번 칸을 빼면 빈 칸 중 다음 2번
  std::vector<bool> usable(12, true);
  usable[1] = false;
  EXPECT_EQ(basket_slot("next", filled, order, true, why, usable), 2);
  EXPECT_EQ(basket_slot("1", filled, order, true, why, usable), 1);  // 칸 번호를 직접 주면 그대로
  std::fill(filled.begin(), filled.end(), "box");
  EXPECT_EQ(basket_slot("next", filled, order, true, why), -1);
  EXPECT_EQ(why, "slot_full");
}

TEST(Tray, GraspHeld) {
  // 퍽이 끼어 덜 닫힘(위치) + 전류. 전류를 모르면(NaN) 위치만
  EXPECT_TRUE(grasp_held(0.005, 0.35, 0.003, 0.25));
  EXPECT_TRUE(grasp_held(0.005, -0.35, 0.003, 0.25));   // 방향 부호 무관
  EXPECT_FALSE(grasp_held(0.0005, 0.35, 0.003, 0.25));  // 끝까지 닫힘 — 빈손
  EXPECT_FALSE(grasp_held(0.005, 0.05, 0.003, 0.25));   // 힘이 안 걸림
  EXPECT_TRUE(grasp_held(0.005, std::nan(""), 0.003, 0.25));
}

TEST(Tray, BoxSpread) {
  // 상자 0.205 각 · 가장자리 0.03 안쪽까지 → 비킬 수 있는 최대 0.0725. 상자 yaw 90°면 축이 돌아감
  auto [x0, y0] = box_spread(0, 0.08, 0.1025, 0.1025, 0.03, 0.0);
  EXPECT_NEAR(x0, 0.0, 1e-9);
  EXPECT_NEAR(y0, 0.0, 1e-9);
  auto [x1, y1] = box_spread(1, 0.08, 0.1025, 0.1025, 0.03, 0.0);  // spread_offset (0, 0.08) → 0.0725로 자름
  EXPECT_NEAR(x1, 0.0, 1e-9);
  EXPECT_NEAR(y1, 0.0725, 1e-9);
  auto [x2, y2] = box_spread(1, 0.08, 0.1025, 0.1025, 0.03, M_PI / 2);
  EXPECT_NEAR(x2, -0.0725, 1e-9);
  EXPECT_NEAR(y2, 0.0, 1e-9);
}

TEST(Tray, Spread) {
  EXPECT_EQ(spread_offset(0, 0.08), std::make_pair(0.0, 0.0));
  EXPECT_EQ(spread_offset(1, 0.08), std::make_pair(0.0, 0.08));
  EXPECT_EQ(spread_offset(2, 0.08), std::make_pair(0.0, -0.08));
  EXPECT_EQ(spread_offset(9, 0.08), std::make_pair(0.0, 0.0));     // 9개째부터 한 칸 바깥 고리 — 0은 가운데 그대로
  EXPECT_EQ(spread_offset(10, 0.08), std::make_pair(0.0, 0.16));
}

TEST(Tray, CarryGoals) {
  const auto g = carry_goals({"11", "12"}, "7");  // 퍽 2개를 적재함에 싣고 장소 7로
  ASSERT_EQ(g.size(), 2u * 3 + 1 + 2u * 2);
  EXPECT_EQ(g[0].skill, "navigate");
  EXPECT_EQ(g[1].skill, "pick");
  EXPECT_EQ(g[1].target, "11");
  EXPECT_EQ(g[2].skill, "place");
  EXPECT_EQ(g[2].target, "tray");
  EXPECT_EQ(g[6].skill, "navigate");
  EXPECT_EQ(g[6].target, "7");
  EXPECT_EQ(g[7].target, "tray");   // 꺼내기
  EXPECT_EQ(g[8].target, "7");      // 장소에 내려놓기
  for (size_t i = 0; i < g.size(); ++i) EXPECT_EQ(g[i].step, i + 1);
}

namespace {
perception::msg::TrackedObject obj(uint32_t id, const std::string & cls, const std::string & color, int tag, double x) {
  perception::msg::TrackedObject o;
  o.id = id;
  o.class_name = cls;
  o.color = color;
  o.pose.pose.position.x = x;
  if (tag >= 0) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "tag";
    kv.value = std::to_string(tag);
    o.details.push_back(kv);
  }
  return o;
}
}  // namespace

TEST(Signal, PickCombo) {
  // MISSION §3: 빨간 퍽(0~3) 중 합 5인 2개 = 2+3 · 답이 여러 개(합 3 = 0+3 · 1+2)면 가까운(cost 작은) 쪽
  EXPECT_EQ(pick_combo({0, 1, 2, 3}, {1, 1, 1, 1}, 2, 5), (std::vector<size_t>{2, 3}));
  EXPECT_EQ(pick_combo({0, 1, 2, 3}, {5, 1, 1, 5}, 2, 3), (std::vector<size_t>{1, 2}));
  EXPECT_TRUE(pick_combo({0, 1, 2, 3}, {1, 1, 1, 1}, 2, 6).empty());  // MISSION §4: 빨강 2개로 6은 불가능
  EXPECT_EQ(pick_combo({4, 5, 6, 7}, {1, 1, 1, 1}, 4, 22).size(), 4u);
}

TEST(Signal, MissionPlan) {
  perception::msg::MissionSignal sig;
  sig.color = "red";
  sig.count = 2;
  sig.sum = 5;
  sig.dest_tag = 13;
  TrackedObjectArray objs;
  objs.objects = {obj(1, "puck", "red", 2, 0.5), obj(2, "puck", "red", -1, 0.6), obj(3, "puck", "green", -1, 0.4),
                  obj(4, "puck", "", -1, 0.9), obj(9, "location_box", "", 13, 2.0)};
  std::set<uint32_t> inspected;
  // 번호 아는 빨간 퍽이 1개뿐 → 번호 모르는 빨간·색 모름 퍽을 가까운 순으로 inspect (초록은 안 봄)
  auto r = signal_plan(objs, sig, 0.0, 0.0, inspected, 4);
  ASSERT_EQ(r.kind, "inspect");
  EXPECT_EQ(r.inspect_ids, (std::vector<uint32_t>{2, 4}));
  EXPECT_EQ(r.goals[0].skill, "inspect");
  // 봤더니 2번 퍽 = 태그 3 → 2+3 = 5 → 옮기기 (목적지 = 장소 상자(태그 13) 물체 9)
  objs.objects[1] = obj(2, "puck", "red", 3, 0.6);
  inspected = {2, 4};
  r = signal_plan(objs, sig, 0.0, 0.0, inspected, 4);
  ASSERT_EQ(r.kind, "carry") << r.why;
  EXPECT_EQ(r.carry_tags, (std::vector<int>{2, 3}));
  EXPECT_EQ(r.goals[1].target, "1");
  EXPECT_EQ(r.goals[4].target, "2");
  EXPECT_EQ(r.goals[6].target, "9");
  // 목적지 태그를 아직 못 봤으면 기다림
  objs.objects.pop_back();
  EXPECT_EQ(signal_plan(objs, sig, 0.0, 0.0, inspected, 4).kind, "wait");
  // 다 봤는데 합을 못 만들면 실패 (say)
  sig.sum = 6;
  EXPECT_EQ(signal_plan(objs, sig, 0.0, 0.0, inspected, 4).kind, "fail");
}

TEST(Signal, VerifyPlan) {
  // 목적지 = 장소 상자(태그 13) 물체 9 (x 2.0) · 옮긴 번호 2·3 · 반경 0.2
  TrackedObjectArray objs;
  objs.objects = {obj(9, "location_box", "", 13, 2.0), obj(1, "puck", "red", 2, 2.05), obj(2, "puck", "red", -1, 1.9),
                  obj(5, "puck", "red", -1, 0.5)};
  // 보러 가기 — 목적지로 navigate · 근처의 번호 모르는 퍽(2)만 inspect (멀리 있는 5는 안 봄)
  auto r = verify_plan(objs, 13, {2, 3}, 0.2, false);
  ASSERT_EQ(r.kind, "look");
  ASSERT_EQ(r.goals.size(), 2u);
  EXPECT_EQ(r.goals[0].target, "9");
  EXPECT_EQ(r.goals[1].skill, "inspect");
  EXPECT_EQ(r.goals[1].target, "2");
  // 봤더니 2번 = 태그 3 → 다 있음
  objs.objects[2] = obj(2, "puck", "red", 3, 1.9);
  EXPECT_EQ(verify_plan(objs, 13, {2, 3}, 0.2, true).kind, "ok");
  // 태그 3 퍽이 다른 곳(0.5)에 있으면 다시 옮김
  objs.objects[2] = obj(2, "puck", "red", 3, 0.5);
  r = verify_plan(objs, 13, {2, 3}, 0.2, true);
  ASSERT_EQ(r.kind, "recarry") << r.why;
  EXPECT_EQ(r.missing, (std::vector<int>{3}));
  EXPECT_EQ(r.goals[1].target, "2");
  EXPECT_EQ(r.goals.back().target, "9");
  // 어디에도 안 보이면 missing · 목적지 태그가 안 보이면 wait
  objs.objects.erase(objs.objects.begin() + 2);
  EXPECT_EQ(verify_plan(objs, 13, {2, 3}, 0.2, true).kind, "missing");
  objs.objects.erase(objs.objects.begin());
  EXPECT_EQ(verify_plan(objs, 13, {2, 3}, 0.2, false).kind, "wait");
}

TEST(Coordinator, ApproachPose) {
  // 로봇 (0,0) · 물체 (0.6, 0) → 물체 앞 0.24 = (0.36, 0), 물체를 바라봄 (yaw 0)
  auto p = plan::approach_pose(0.6, 0.0, 0.0, 0.0, 0.24);
  EXPECT_NEAR(p.position.x, 0.36, 1e-9);
  EXPECT_NEAR(p.position.y, 0.0, 1e-9);
  EXPECT_NEAR(2 * std::atan2(p.orientation.z, p.orientation.w), 0.0, 1e-9);
  // 로봇이 물체의 +y 쪽 → 그쪽에서 다가가 −y를 바라봄 (예전엔 늘 물체 −x 쪽 · yaw 0)
  p = plan::approach_pose(1.0, 0.0, 1.0, 1.0, 0.24);
  EXPECT_NEAR(p.position.x, 1.0, 1e-9);
  EXPECT_NEAR(p.position.y, 0.24, 1e-9);
  EXPECT_NEAR(2 * std::atan2(p.orientation.z, p.orientation.w), -M_PI / 2, 1e-9);
  // 로봇이 물체 바로 위면 −x 쪽에서
  p = plan::approach_pose(1.0, 2.0, 1.0, 2.0, 0.3);
  EXPECT_NEAR(p.position.x, 0.7, 1e-9);
  EXPECT_NEAR(p.position.y, 2.0, 1e-9);
}

// stand (누운 퍽 세우기) · pick 누운 퍽 거절 — 대상 확인은 MoveIt 서버 대기보다 먼저라 서버 없이도 이유가 나옴
namespace {
perception::msg::TrackedObject puck_with(uint32_t id, const std::string & posture, bool yaw_known) {
  perception::msg::TrackedObject o;
  o.id = id;
  o.class_name = posture == "lying" ? "puck_lying" : "puck";
  o.pose.pose.orientation.w = 1.0;
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "posture";
  kv.value = posture;
  o.details.push_back(kv);
  if (yaw_known) {
    kv.key = "yaw_period";
    kv.value = "180";
    o.details.push_back(kv);
  }
  return o;
}

// 한 단계짜리 계획을 보내고 그 계획의 FAILED 이유를 받음 ("" = 시간 안에 실패 없음)
std::string fail_reason(const std::string & skill, const std::string & target, const perception::msg::TrackedObject & o,
                        uint32_t plan_id) {
  auto node = std::make_shared<Coordinator>();
  auto probe = std::make_shared<rclcpp::Node>("probe_stand_" + std::to_string(plan_id));
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  auto objects_pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto goals_pub = probe->create_publisher<SubGoalArray>("/task/sub_goals", latched);
  std::string reason;
  auto sub = probe->create_subscription<TaskStatus>("/task/status", latched,
    [&](TaskStatus::SharedPtr m) { if (m->plan_id == plan_id && m->state == TaskStatus::FAILED) reason = m->reason; });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  SubGoalArray plan;
  plan.plan_id = plan_id;
  SubGoal g;
  g.step = 1;
  g.skill = skill;
  g.target = target;
  plan.goals.push_back(g);
  TrackedObjectArray objects;
  objects.objects.push_back(o);
  goals_pub->publish(plan);
  spin_until(exec, [&]() { objects_pub->publish(objects); }, [&]() { return !reason.empty(); });
  return reason;
}
}  // namespace

TEST_F(RosTest, StandRejectsStandingPuck) {
  EXPECT_EQ(fail_reason("stand", "5", puck_with(5, "standing", true), 21), "stand_not_lying");
}

TEST_F(RosTest, StandNeedsAxis) {
  EXPECT_EQ(fail_reason("stand", "5", puck_with(5, "lying", false), 22), "stand_no_axis");
}

TEST_F(RosTest, StandIsAllowedSkill) {
  // 세울 수 있는 대상이면 계획 수락 → MoveIt 서버를 기다리는 중 (invalid_plan이 아님 · 2초 안엔 실패 없음)
  auto node = std::make_shared<Coordinator>();
  node->set_parameter(rclcpp::Parameter("server_wait_s", 30.0));
  auto probe = std::make_shared<rclcpp::Node>("probe_stand_ok");
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  auto objects_pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto goals_pub = probe->create_publisher<SubGoalArray>("/task/sub_goals", latched);
  std::vector<TaskStatus> got;
  auto sub = probe->create_subscription<TaskStatus>("/task/status", latched,
    [&](TaskStatus::SharedPtr m) { if (m->plan_id == 23) got.push_back(*m); });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  SubGoalArray plan;
  plan.plan_id = 23;
  SubGoal g;
  g.step = 1;
  g.skill = "stand";
  g.target = "5";
  plan.goals.push_back(g);
  TrackedObjectArray objects;
  objects.objects.push_back(puck_with(5, "lying", true));
  goals_pub->publish(plan);
  ASSERT_TRUE(spin_until(exec, [&]() { objects_pub->publish(objects); },
    [&]() { for (auto & s : got) if (s.state == TaskStatus::RUNNING) return true; return false; }));
  spin_until(exec, [&]() { objects_pub->publish(objects); }, [] { return false; }, 2.0);
  for (auto & s : got) EXPECT_NE(s.state, TaskStatus::FAILED) << s.reason;
}

TEST_F(RosTest, PickRejectsLyingPuck) {
  EXPECT_EQ(fail_reason("pick", "5", puck_with(5, "lying", true), 24), "pick_lying");
}
