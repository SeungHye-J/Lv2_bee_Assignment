#pragma once

// 모니터 매니저 (module5 fake_camera_bringup/monitor_manager.cpp 그대로): 떠 있는 토픽을 파일로 떨궈
// 호스트(통제실·시뮬레이션)에서 보게 한다. 반대 방향(호스트 → 토픽)은 input_manager.
// 0.5초마다 그래프를 훑어 새 토픽을 자동 구독 (토픽·타입을 코드에 적지 않음 → 추가돼도 재빌드 불필요).
//   <debug_dir>/topic/<토픽명>/message    마지막 메시지 JSON {"type": "...", "data": {...}} (덮어씀)
//   <debug_dir>/topic/<토픽명>/image.jpg  Image / CompressedImage 토픽이면 마지막 영상 (덮어씀)
//   <debug_dir>/topic/<토픽명>/depth.png  16UC1 depth 영상이면 무손실 16비트 PNG [mm] (시뮬레이터 깊이·점구름 보기)
//   예) /task_status → /ws/debug/topic/task_status/message (호스트 lv2_module6/debug/topic/task_status/message)
// test-logger의 echo·info와 같은 폴더. 갱신 시각은 파일 mtime으로 본다.
// image.jpg는 토픽별로 period_s에 한 번만 (카메라 30fps 영상을 다 쓰면 디스크만 바쁨).
//
// 파라미터: debug_dir(기본 /ws/debug — 그 아래 topic/에 저장), period_s(이미지 저장 간격),
//   raw_images (기본 true): false면 sensor_msgs/Image(무압축 영상)는 구독 자체를 안 함 (네트워크 너머에서 띄울 때)
//   exclude_topics (기본 []): 이름으로 뺄 토픽 목록
//
// 메시지 → JSON: rosx_introspection(PlotJuggler가 쓰는 라이브러리)이 .msg 정의로 런타임 파싱.
//   타입 없이 바이트로 받는 GenericSubscription + Parser::deserializeIntoJson.
//   100개 넘는 배열(영상 픽셀 등)은 JSON에서 빠진다 (DISCARD_LARGE_ARRAYS 기본값).
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace faker {

namespace fs = std::filesystem;

class MonitorManager : public rclcpp::Node {
public:
  MonitorManager();

private:
  void report();

  void scan();

  // 이미지: best-effort 구독은 reliable·best-effort 발행 모두와 연결된다
  static rclcpp::QoS qos();

  // 메시지: 발행자 QoS에 맞춰 구독 (ros2 topic echo와 같은 방식).
  // 발행자가 모두 transient_local이면 transient_local로 → 늦게 붙어도 저장된 마지막 값을 바로 받음
  //   (예: /task_status·/sub_goals는 바뀔 때만 발행 → volatile이면 다음 발행까지 아무것도 못 받음)
  // 발행자가 모두 reliable일 때만 reliable (best-effort 발행자가 있으면 reliable 구독은 연결 안 됨)
  rclcpp::QoS qos_for(const std::string & topic);

  void subscribe_message(const std::string & topic, const std::string & type);

  void subscribe_image(const std::string & topic, const std::string & type);

  bool due(const std::string & key);

  fs::path dir_of(const std::string & topic) const;

  // 임시 파일에 쓰고 rename → 호스트가 반쯤 쓰인 파일을 읽지 않게.
  // 폴더는 매번 만든다 (test-logger가 시작할 때 debug/topic을 지움).
  static fs::path prepare_tmp(const fs::path & out);

  void write_file(const fs::path & out, const std::string & text);

  void save_image(const std::string & topic, const cv::Mat & image);

  void save_depth(const std::string & topic, const cv::Mat & depth);

  std::string debug_dir_;
  bool raw_images_;
  std::set<std::string> excluded_;
  double period_;
  std::set<std::string> seen_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::map<std::string, rclcpp::Time> last_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  size_t written_ = 0;         // 최근 5초 저장 건수 (report마다 초기화)
  size_t write_failed_ = 0;
  size_t excluded_count_ = 0;
};

}  // namespace faker
