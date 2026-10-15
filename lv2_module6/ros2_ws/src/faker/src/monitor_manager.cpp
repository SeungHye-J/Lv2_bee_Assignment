// monitor_manager 구현 — 설명·선언은 include/faker/monitor_manager.hpp, main은 monitor_manager_main.cpp
#include "faker/monitor_manager.hpp"

#include <sys/stat.h>

namespace faker {

MonitorManager::MonitorManager()
: rclcpp::Node("monitor_manager") {
  // debug 폴더를 받는다. 토픽별 파일은 <debug_dir>/topic/<토픽>/ 에 저장 (통제실이 읽는 위치)
  debug_dir_ = declare_parameter<std::string>("debug_dir", "/ws/debug");
  period_ = declare_parameter<double>("period_s", 0.2);
  raw_images_ = declare_parameter<bool>("raw_images", true);
  const auto excluded = declare_parameter<std::vector<std::string>>("exclude_topics", std::vector<std::string>{});
  excluded_ = std::set<std::string>(excluded.begin(), excluded.end());

  // 저장 폴더를 시작할 때 만든다 (없으면 생성). 못 만들면 바로 알 수 있게 종료
  const fs::path topic_dir = fs::absolute(fs::path(debug_dir_) / "topic");
  std::error_code ec;
  fs::create_directories(topic_dir, ec);
  if (ec || !fs::is_directory(topic_dir)) {
    RCLCPP_FATAL(get_logger(), "저장 폴더를 만들 수 없음: %s (%s) — 경로·권한 확인 (root 소유면 chown)",
      topic_dir.c_str(), ec.message().c_str());
    throw std::runtime_error("debug_dir 생성 실패");
  }
  RCLCPP_INFO(get_logger(), "시작 | 저장: %s | ROS_DOMAIN_ID=%zu | raw_images=%s | period_s=%.2f | 제외 %zu개",
    topic_dir.c_str(), get_node_base_interface()->get_context()->get_domain_id(),
    raw_images_ ? "true" : "false", period_, excluded_.size());

      scan_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { scan(); });
  // 5초마다 상태 로그 — 실행 중인지, 실제로 저장되고 있는지 확인용
  status_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { report(); });
  scan();
}

void MonitorManager::report() {
  if (subs_.empty()) {
    RCLCPP_WARN(get_logger(), "구독할 토픽 없음 — 발행 노드가 떠 있는지, ROS_DOMAIN_ID(현재 %zu)·네트워크 확인",
      get_node_base_interface()->get_context()->get_domain_id());
  } else if (written_ == 0) {
    RCLCPP_WARN(get_logger(), "토픽 %zu개 구독 중, 최근 5초 저장 0건 — 발행이 멈췄거나 QoS 불일치",
      seen_.size() - excluded_count_);
  } else {
    RCLCPP_INFO(get_logger(), "토픽 %zu개 구독 중, 최근 5초 저장 %zu건%s",
      seen_.size() - excluded_count_, written_, write_failed_ ? " (쓰기 실패 있음)" : "");
  }
  written_ = 0;
  write_failed_ = 0;
}

void MonitorManager::scan() {
  for (const auto & [topic, types] : get_topic_names_and_types()) {
    // 숨김 토픽(이름 구간이 '_'로 시작, ros2 topic list에도 안 나옴) 제외.
    // Lyrical은 구독마다 <토픽>/_buf_cpu 를 만들어서 이걸 또 구독하면 끝없이 늘어난다.
    // /rosout·/parameter_events는 노드만 있으면 항상 있는 시스템 토픽 → 제외 (test-logger와 같은 규칙)
    if (types.empty() || seen_.count(topic) || topic.find("/_") != std::string::npos ||
      topic == "/rosout" || topic == "/parameter_events")
    {
      continue;
    }
    seen_.insert(topic);
    if (excluded_.count(topic) || (!raw_images_ && types.front() == "sensor_msgs/msg/Image")) {
      RCLCPP_INFO(get_logger(), "제외: %s (%s)", topic.c_str(), types.front().c_str());
      ++excluded_count_;
      continue;
    }
    types_[topic] = types.front();
    write_topic_list();
    subscribe_message(topic, types.front());
    subscribe_image(topic, types.front());
  }
}

void MonitorManager::write_topic_list() {
  std::string list = "{\"topics\": [";
  bool first = true;
  for (const auto & [name, type] : types_) {
    if (!first) list += ", ";
    first = false;
    list += "{\"name\": \"" + name + "\", \"type\": \"" + type + "\"}";
  }
  list += "]}\n";
  write_file(fs::path(debug_dir_) / "topic" / "list.json", list);
}

rclcpp::QoS MonitorManager::qos() { return rclcpp::SensorDataQoS(); }

rclcpp::QoS MonitorManager::qos_for(const std::string & topic) {
  const auto pubs = get_publishers_info_by_topic(topic);
  const auto all = [&](auto pred) {
    return !pubs.empty() && std::all_of(pubs.begin(), pubs.end(), pred);
  };
  rclcpp::QoS q = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  if (all([](const auto & p) { return p.qos_profile().reliability() == rclcpp::ReliabilityPolicy::Reliable; })) {
    q.reliable();
  }
  if (all([](const auto & p) {
      return p.qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal;
    }))
  {
    q.transient_local();
  }
  return q;
}

void MonitorManager::subscribe_message(const std::string & topic, const std::string & type) {
  std::shared_ptr<RosMsgParser::Parser> parser;
  try {
    parser = std::make_shared<RosMsgParser::Parser>(
      topic, RosMsgParser::ROSType(type), RosMsgParser::GetMessageDefinition(type));
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "message 저장 불가 %s (%s): %s", topic.c_str(), type.c_str(), e.what());
    return;
  }
  const auto msg_qos = qos_for(topic);
  subs_.push_back(create_generic_subscription(topic, type, msg_qos,
    [this, topic, type, parser](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
      const auto & raw = msg->get_rcl_serialized_message();
      RosMsgParser::NanoCDR_Deserializer deserializer;
      std::string data;
      try {
        parser->deserializeIntoJson({raw.buffer, raw.buffer_length}, &data, &deserializer, 0, true);
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s JSON 변환 실패: %s",
          topic.c_str(), e.what());
        return;
      }
      write_file(dir_of(topic) / "message", "{\"type\": \"" + type + "\", \"data\": " + data + "}\n");
    }));
  RCLCPP_INFO(get_logger(), "message 저장 시작: %s (%s, %s%s)", topic.c_str(), type.c_str(),
    msg_qos.reliability() == rclcpp::ReliabilityPolicy::Reliable ? "reliable" : "best-effort",
    msg_qos.durability() == rclcpp::DurabilityPolicy::TransientLocal ? ", transient_local" : "");
}

void MonitorManager::subscribe_image(const std::string & topic, const std::string & type) {
  if (type == "sensor_msgs/msg/CompressedImage") {
    subs_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(topic, qos(),
      [this, topic](const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg) {
        if (due(topic + "#image")) {
          // NOTE (Lyrical): data가 std::vector가 아님(rosidl_buffer) → 복사 없이 Mat으로 감싸 디코드
          const cv::Mat buf(1, static_cast<int>(msg->data.size()), CV_8UC1,
            const_cast<uint8_t *>(msg->data.data()));
          save_image(topic, cv::imdecode(buf, cv::IMREAD_UNCHANGED));
        }
      }));
  } else if (type == "sensor_msgs/msg/Image") {
    subs_.push_back(create_subscription<sensor_msgs::msg::Image>(topic, qos(),
      [this, topic](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
        if (!due(topic + "#image")) {
          return;
        }
        try {
          if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
            msg->encoding == sensor_msgs::image_encodings::MONO16)
          {
            save_depth(topic, cv_bridge::toCvShare(msg)->image);  // jpg로는 깊이 값이 깨진다
            return;
          }
          // mono8 마스크 등은 그대로, 컬러는 BGR로 맞춰 저장
          const bool mono = sensor_msgs::image_encodings::numChannels(msg->encoding) == 1;
          save_image(topic, cv_bridge::toCvShare(msg, mono ? "mono8" : "bgr8")->image);
        } catch (const std::exception & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 변환 실패: %s",
            topic.c_str(), e.what());
        }
      }));
  } else {
    return;
  }
  RCLCPP_INFO(get_logger(), "image 저장 시작: %s", topic.c_str());
}

bool MonitorManager::due(const std::string & key) {
  const auto t = now();
  auto it = last_.find(key);
  if (it != last_.end() && (t - it->second).seconds() < period_) {
    return false;
  }
  last_[key] = t;
  return true;
}

fs::path MonitorManager::dir_of(const std::string & topic) const {
  return fs::path(debug_dir_) / "topic" / topic.substr(1);
}

fs::path MonitorManager::prepare_tmp(const fs::path & out) {
  std::error_code ec;
  fs::create_directories(out.parent_path(), ec);
  ::chmod(out.parent_path().c_str(), 0777);  // 컨테이너(root)가 만들어도 호스트가 rename으로 교체 가능
  return out.parent_path() / (".tmp-" + out.filename().string());
}

void MonitorManager::write_file(const fs::path & out, const std::string & text) {
  const fs::path tmp = prepare_tmp(out);
  std::ofstream file(tmp);
  file << text;
  file.close();
  std::error_code ec;
  if (!file || (fs::rename(tmp, out, ec), ec)) {
    ++write_failed_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "쓰기 실패: %s — 권한·디스크 확인", out.c_str());
    return;
  }
  ++written_;
}

void MonitorManager::save_image(const std::string & topic, const cv::Mat & image) {
  if (image.empty()) {
    return;
  }
  const fs::path out = dir_of(topic) / "image.jpg";
  const fs::path tmp = prepare_tmp(out);  // 확장자 .jpg 유지 → imwrite가 형식 판별
  std::error_code ec;
  if (!cv::imwrite(tmp.string(), image) || (fs::rename(tmp, out, ec), ec)) {
    ++write_failed_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "이미지 쓰기 실패: %s", out.c_str());
    return;
  }
  ++written_;
}

void MonitorManager::save_depth(const std::string & topic, const cv::Mat & depth) {
  const fs::path out = dir_of(topic) / "depth.png";
  const fs::path tmp = prepare_tmp(out);
  std::error_code ec;
  if (!cv::imwrite(tmp.string(), depth) || (fs::rename(tmp, out, ec), ec)) {
    ++write_failed_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "depth 쓰기 실패: %s", out.c_str());
    return;
  }
  ++written_;
}

}  // namespace faker
