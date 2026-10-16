// vlm_task_planner 구현 — 노드 설명·입출력·멤버 선언은 include/plan/vlm_task_planner.hpp, main은 vlm_task_planner_main.cpp
#include "plan/vlm_task_planner.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <thread>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace plan {

namespace {

// base64 (요청 본문 images[].data_b64용) — 의존성 없이 직접 인코딩
std::string base64_encode(const std::vector<uint8_t> & in) {
  static const char * tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string o;
  o.reserve((in.size() + 2) / 3 * 4);
  for (size_t i = 0; i < in.size(); i += 3) {
    const uint32_t a = in[i], b = i + 1 < in.size() ? in[i + 1] : 0, c = i + 2 < in.size() ? in[i + 2] : 0;
    const uint32_t v = (a << 16) | (b << 8) | c;
    o += tbl[(v >> 18) & 63];
    o += tbl[(v >> 12) & 63];
    o += (i + 1 < in.size()) ? tbl[(v >> 6) & 63] : '=';
    o += (i + 2 < in.size()) ? tbl[v & 63] : '=';
  }
  return o;
}

// JSON 문자열 이스케이프 (요청 본문용)
std::string json_escape(const std::string & s) {
  std::string o;
  o.reserve(s.size() + 2);
  for (unsigned char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if (c < 0x20) {
          char b[8];
          snprintf(b, sizeof(b), "\\u%04x", c);
          o += b;
        } else { o += static_cast<char>(c); }
    }
  }
  return o;
}

// "http://host:port"·"host:port" → host/port (https 미지원 — 로컬 http만)
bool split_endpoint(const std::string & ep, std::string & host, int & port) {
  std::string t = ep;
  if (t.rfind("https://", 0) == 0) return false;
  if (t.rfind("http://", 0) == 0) t.erase(0, 7);
  const auto c = t.find(':');
  if (c == std::string::npos) { host = t; port = 80; }
  else {
    host = t.substr(0, c);
    port = std::atoi(t.substr(c + 1).c_str());
  }
  return !host.empty() && port > 0 && port < 65536;
}

bool wait_fd(int fd, bool for_write, double timeout_s) {
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd, &set);
  timeval tv{static_cast<long>(timeout_s), 0};
  const int r = select(fd + 1, for_write ? nullptr : &set, for_write ? &set : nullptr, nullptr, &tv);
  return r > 0;
}

// blocking HTTP POST — 200이면 body. select 타임아웃 (워커 스레드에서만 호출)
bool http_post(const std::string & host, int port, const std::string & body,
  std::string & out, double timeout_s) {
  addrinfo hints{};
  hints.ai_socktype = SOCK_STREAM;
  addrinfo * list = nullptr;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &list) != 0 || !list) return false;
  int fd = -1;
  for (addrinfo * a = list; a; a = a->ai_next) {
    fd = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
    if (fd < 0) continue;
    fcntl(fd, F_SETFL, O_NONBLOCK);
    if (connect(fd, a->ai_addr, a->ai_addrlen) != 0 && errno != EINPROGRESS) {
      close(fd);
      fd = -1;
      continue;
    }
    break;
  }
  freeaddrinfo(list);
  if (fd < 0 || !wait_fd(fd, true, timeout_s)) {
    if (fd >= 0) close(fd);
    return false;
  }
  int err = 0;
  socklen_t len = sizeof(err);
  if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0) {
    close(fd);
    return false;
  }
  const std::string req = "POST /plan HTTP/1.0\r\nHost: " + host + "\r\nContent-Type: application/json\r\n"
    "Content-Length: " + std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  size_t sent = 0;
  while (sent < req.size()) {
    if (std::chrono::steady_clock::now() > deadline) { close(fd); return false; }
    if (!wait_fd(fd, true, 1.0)) continue;
    const ssize_t n = send(fd, req.data() + sent, req.size() - sent, MSG_NOSIGNAL);
    if (n <= 0) { close(fd); return false; }
    sent += static_cast<size_t>(n);
  }
  std::string raw;
  char buf[4096];
  for (;;) {
    if (std::chrono::steady_clock::now() > deadline) { close(fd); return false; }
    if (!wait_fd(fd, false, 1.0)) continue;  // idle — 아직 EOF 아님
    const ssize_t n = recv(fd, buf, sizeof(buf), 0);
    if (n == 0) break;  // 서버가 닫음 (HTTP/1.0 + Connection: close)
    if (n < 0) { close(fd); return false; }
    raw.append(buf, static_cast<size_t>(n));
  }
  close(fd);
  const auto eol = raw.find("\r\n");
  if (eol == std::string::npos || raw.substr(0, eol).find(" 200") == std::string::npos) return false;
  const auto h = raw.find("\r\n\r\n");
  if (h == std::string::npos) return false;
  out = raw.substr(h + 4);
  return true;
}

// {"ok": true, "goals": [{step,skill,target,relation,offset_m}]} 최소 파서
struct Cursor {
  const char * p;
  void ws() { while (*p && std::isspace(static_cast<unsigned char>(*p))) ++p; }
  bool str(std::string & o);
  bool num(double & o);
  bool skip();  // 값 하나 건너뜀 (중첩 포함)
};

bool Cursor::str(std::string & o) {
  if (*p != '"') return false;
  ++p;
  o.clear();
  while (*p && *p != '"') {
    if (*p == '\\') {
      ++p;
      switch (*p) {
        case '"': o += '"'; break;
        case '\\': o += '\\'; break;
        case 'n': o += '\n'; break;
        case 't': o += '\t'; break;
        case 'r': o += '\r'; break;
        default: if (*p) o += *p; break;
      }
      if (*p) ++p;
    } else { o += *p++; }
  }
  if (*p != '"') return false;
  ++p;
  return true;
}

bool Cursor::num(double & o) {
  char * end = nullptr;
  o = std::strtod(p, &end);
  if (end == p) return false;
  p = end;
  return true;
}

bool Cursor::skip() {
  ws();
  if (*p == '"') {
    std::string d;
    return str(d);
  }
  if (*p == '{' || *p == '[') {
    const char open = *p, close = (open == '{' ? '}' : ']');
    ++p;
    ws();
    if (*p == close) { ++p; return true; }
    for (;;) {
      if (open == '{') {
        std::string k;
        if (!str(k)) return false;
        ws();
        if (*p != ':') return false;
        ++p;
      }
      if (!skip()) return false;
      ws();
      if (*p == ',') { ++p; continue; }
      if (*p == close) { ++p; return true; }
      return false;
    }
  }
  if (!std::strncmp(p, "true", 4)) { p += 4; return true; }
  if (!std::strncmp(p, "false", 5)) { p += 5; return true; }
  if (!std::strncmp(p, "null", 4)) { p += 4; return true; }
  double d;
  return num(d);
}

bool parse_goals(const std::string & body, std::vector<SubGoal> & goals) {
  Cursor c{body.c_str()};
  c.ws();
  if (*c.p != '{') return false;
  const char * ok = std::strstr(c.p, "\"ok\"");
  if (!ok) return false;
  c.p = ok + 4;
  c.ws();
  if (*c.p != ':') return false;
  ++c.p;
  c.ws();
  if (std::strncmp(c.p, "true", 4) != 0) return false;  // ok:false → 실패 취급
  const char * g = std::strstr(c.p, "\"goals\"");
  if (!g) return false;
  c.p = g + 7;
  c.ws();
  if (*c.p != ':') return false;
  ++c.p;
  c.ws();
  if (*c.p != '[') return false;
  ++c.p;
  static const std::set<std::string> allowed = {"navigate", "pick", "place", "say"};
  uint32_t step = 0;
  for (;;) {
    c.ws();
    if (*c.p == ']') return !goals.empty();
    if (*c.p != '{') return false;
    ++c.p;
    SubGoal sg;
    uint32_t got_step = 0;
    bool has_step = false, has_skill = false;
    for (;;) {
      c.ws();
      if (*c.p == '}') { ++c.p; break; }
      std::string k;
      if (!c.str(k)) return false;
      c.ws();
      if (*c.p != ':') return false;
      ++c.p;
      c.ws();
      if (k == "step") {
        double v;
        if (!c.num(v)) return false;
        got_step = static_cast<uint32_t>(v);
        has_step = true;
      } else if (k == "skill" || k == "target" || k == "relation") {
        std::string v;
        if (*c.p == '"') {
          if (!c.str(v)) return false;
        } else {  // target이 숫자면 문자열로
          double n;
          if (!c.num(n)) return false;
          v = std::to_string(static_cast<long long>(n));
        }
        if (k == "skill") { sg.skill = v; has_skill = true; }
        else if (k == "target") sg.target = v;
        else sg.relation = v;
      } else if (k == "offset_m") {
        double v;
        if (!c.num(v)) return false;
        sg.offset_m = static_cast<float>(v);
      } else {
        if (!c.skip()) return false;
      }
      c.ws();
      if (*c.p == ',') { ++c.p; continue; }
      if (*c.p == '}') continue;  // 루프 head에서 닫음
      return false;
    }
    if (!has_step || !has_skill || got_step != ++step || !allowed.count(sg.skill)) return false;
    sg.step = got_step;
    goals.push_back(sg);
    c.ws();
    if (*c.p == ',') { ++c.p; continue; }
    if (*c.p == ']') return !goals.empty();
    return false;
  }
}

}  // namespace

VlmTaskPlanner::VlmTaskPlanner()
: rclcpp::Node("vlm_task_planner") {
  max_replans_ = declare_parameter<int>("max_replans", 1);
  vlm_endpoint_ = declare_parameter<std::string>("vlm_endpoint", "");
  vlm_timeout_s_ = declare_parameter<double>("vlm_timeout_s", 120.0);
  goals_pub_ = create_publisher<SubGoalArray>("/sub_goals", rclcpp::QoS(10).reliable().transient_local());
  cmd_sub_ = create_subscription<std_msgs::msg::String>("/user_command", rclcpp::QoS(10).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) {
      RCLCPP_INFO(get_logger(), "명령: \"%s\"", m->data.c_str());
      replans_ = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_ = m->data;
        replan_ = ReplanInfo{};
      }
      pending_ = true;
      make_plan();
    });
  objects_sub_ = create_subscription<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](TrackedObjectArray::ConstSharedPtr m) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        objects_ = m;
      }
      if (pending_) make_plan();  // 물체 목록보다 명령이 먼저 왔으면 지금 계획
    });
  status_sub_ = create_subscription<TaskStatus>("/task_status", rclcpp::QoS(10).reliable().transient_local(),
    [this](TaskStatus::ConstSharedPtr m) { on_status(*m); });
  const auto sensor_qos = rclcpp::SensorDataQoS();
  for (const std::string cam : {"pole_cam", "gripper_cam"}) {
    image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
      "/" + cam + "/color/image_raw", sensor_qos,
      [this, cam](sensor_msgs::msg::Image::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        images_[cam] = m;  // 최신 1장만 유지 (VLM 호출 때 꺼냄)
      }));
  }
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    if (pending_) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "계획 보류 — 대상 물체 대기");
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, plan_id_ ? "계획 " + std::to_string(plan_id_.load()) : "명령 대기");
    s.add("plan_id", plan_id_.load());
    s.add("재계획", std::to_string(replans_) + "/" + std::to_string(max_replans_));
    s.add("보이는 물체", objects_ ? objects_->objects.size() : 0);
    s.add("vlm", vlm_endpoint_.empty() ? std::string("template") : vlm_endpoint_);
    s.add("vlm 비행", vlm_busy_.load() ? "예" : "아니오");
  });
}

void VlmTaskPlanner::on_status(const TaskStatus & s) {
  if (s.state != TaskStatus::FAILED || s.plan_id != plan_id_.load()) return;
  if (s.reason.rfind("manual", 0) == 0) return;  // 수동 개입으로 멈춘 것 — 사람이 다시 명령할 때까지
  if (replans_ >= max_replans_) return;  // coordinator가 알리고 정지한다
  ++replans_;
  RCLCPP_WARN(get_logger(), "step %u 실패(%s) → 재계획 %d/%d", s.step, s.reason.c_str(), replans_, max_replans_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    replan_ = ReplanInfo{s.plan_id, s.step, s.reason, true};
  }
  pending_ = true;
  make_plan();  // VLM이면 실패 이유를 프롬프트에 넣어 다른 접근 방향으로
}

std::string VlmTaskPlanner::find_in(const TrackedObjectArray & objs, const std::string & cls) {
  for (const auto & o : objs.objects) if (o.class_name == cls) return std::to_string(o.id);
  return "";
}

void VlmTaskPlanner::make_plan() {
  TrackedObjectArray::ConstSharedPtr objs;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    objs = objects_;
  }
  if (!objs) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "/tracked_objects 대기 — 계획 보류");
    return;
  }
  if (vlm_endpoint_.empty()) {  // 템플릿(오프라인)은 퍽·목표 id가 있어야 짜진다
    if (find_in(*objs, "puck").empty() || find_in(*objs, "goal_zone").empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "/tracked_objects에 퍽·목표 구역이 없음 — 계획 보류");
      return;
    }
    make_template(*objs);
    return;
  }
  pending_ = true;  // VLM이면 인지 유무와 무관하게 사용자 명령 그대로 전달 (빈 물체+이미지로 판단은 모델 몫)
  try_vlm_async();
}

void VlmTaskPlanner::make_template(const TrackedObjectArray & objs) {
  const auto puck = find_in(objs, "puck"), goal_zone = find_in(objs, "goal_zone");
  if (puck.empty() || goal_zone.empty()) return;  // 물체 사라짐 — 보류 유지
  pending_ = false;
  SubGoalArray plan;
  plan.header.stamp = now();
  plan.plan_id = plan_id_.fetch_add(1) + 1;
  plan.goals.push_back(goal(1, "navigate", puck, "front", 0.20f));
  plan.goals.push_back(goal(2, "pick", puck, "", 0.0f));
  plan.goals.push_back(goal(3, "navigate", goal_zone, "front", 0.20f));
  plan.goals.push_back(goal(4, "place", goal_zone, "on", 0.0f));
  goals_pub_->publish(plan);
  RCLCPP_INFO(get_logger(), "/sub_goals plan=%u (%zu steps) [template]", plan.plan_id, plan.goals.size());
}

void VlmTaskPlanner::try_vlm_async() {
  bool expected = false;
  if (!vlm_busy_.compare_exchange_strong(expected, true)) return;  // 비행 중 — 끝난 워커가 최신으로 다시 시도
  const uint64_t epoch = epoch_.fetch_add(1) + 1;
  std::string cmd;
  TrackedObjectArray objs;
  ReplanInfo rp;
  std::vector<ImageFrame> imgs;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd = cmd_;
    if (objects_) objs = *objects_;
    rp = replan_;
    for (const auto & cam : {"pole_cam", "gripper_cam"}) {
      auto it = images_.find(cam);
      if (it != images_.end() && it->second) imgs.push_back({cam, it->second});
    }
  }
  std::thread([this, epoch, cmd, objs, rp, imgs]() {
    std::vector<SubGoal> goals;
    const bool ok = fetch_goals(vlm_endpoint_, plan_request(cmd, objs, rp, imgs), goals, vlm_timeout_s_);
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stale = (epoch != epoch_.load());
      // 발행 결정까지 잠금 — make_plan의 epoch 증가와 직렬화 (낡은 계획 이중 발행 방지)
      if (!stale) {
        vlm_busy_ = false;
        if (!ok) {
          RCLCPP_WARN(get_logger(), "VLM 실패 — 템플릿 폴백");
          make_template(objs);
        } else {
          pending_ = false;
          SubGoalArray plan;
          plan.header.stamp = now();
          plan.plan_id = plan_id_.fetch_add(1) + 1;
          plan.goals = goals;
          goals_pub_->publish(plan);
          RCLCPP_INFO(get_logger(), "/sub_goals plan=%u (%zu steps) [vlm]", plan.plan_id, plan.goals.size());
        }
      } else {
        vlm_busy_ = false;
      }
    }
    if (stale && pending_.load()) try_vlm_async();  // 최신 스냅샷으로 다시
  }).detach();
}

// sensor_msgs/Image(rgb8) → 320px JPEG base64 (ai-service images[]용)
std::string VlmTaskPlanner::encode_jpeg_base64(const sensor_msgs::msg::Image & msg) {
  if (msg.encoding != "rgb8" || msg.width == 0 || msg.height == 0) return "";
  const size_t expect = static_cast<size_t>(msg.width) * msg.height * 3;
  if (msg.data.size() < expect) return "";
  cv::Mat rgb(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
  cv::Mat small;
  const double scale = 320.0 / msg.width;
  cv::resize(rgb, small, cv::Size(), scale, scale, cv::INTER_LINEAR);
  std::vector<uint8_t> jpeg;
  if (!cv::imencode(".jpg", small, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70})) return "";
  if (jpeg.size() > 500 * 1024) return "";  // ai-service 장당 제한
  return base64_encode(jpeg);
}

std::string VlmTaskPlanner::plan_request(
  const std::string & cmd, const TrackedObjectArray & objs, const ReplanInfo & rp,
  const std::vector<ImageFrame> & imgs) {
  std::string o = "{\"command\": \"" + json_escape(cmd) + "\", \"objects\": [";
  bool first = true;
  for (const auto & obj : objs.objects) {
    if (!first) o += ", ";
    first = false;
    char pos[128], size[128];
    snprintf(pos, sizeof(pos), "[%.3f, %.3f, %.3f]",
      obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z);
    snprintf(size, sizeof(size), "[%.3f, %.3f, %.3f]", obj.size.x, obj.size.y, obj.size.z);
    char conf[16];
    snprintf(conf, sizeof(conf), "%.2f", obj.confidence);
    char px[64];
    snprintf(px, sizeof(px), "[%d, %d]", obj.px[0], obj.px[1]);
    o += std::string("{\"id\": ") + std::to_string(obj.id) + ", \"class_name\": \"" +
      json_escape(obj.class_name) + "\", \"position\": " + pos + ", \"size\": " + size +
      ", \"confidence\": " + conf + ", \"camera\": \"" + json_escape(obj.camera) +
      "\", \"px\": " + px + "}";
  }
  o += "]";
  if (rp.active) {
    o += ", \"replan\": {\"plan_id\": " + std::to_string(rp.plan_id) + ", \"failed_step\": " +
      std::to_string(rp.step) + ", \"reason\": \"" + json_escape(rp.reason) + "\"}";
  }
  o += ", \"images\": [";
  bool ifirst = true;
  for (const auto & im : imgs) {
    if (!im.msg) continue;
    const std::string b64 = encode_jpeg_base64(*im.msg);
    if (b64.empty()) continue;
    if (!ifirst) o += ", ";
    ifirst = false;
    o += "{\"camera\": \"" + im.camera + "\", \"format\": \"jpeg\", \"data_b64\": \"" + b64 + "\"}";
  }
  o += "]";
  return o + "}";
}

bool VlmTaskPlanner::fetch_goals(const std::string & endpoint, const std::string & body,
  std::vector<SubGoal> & goals, double timeout_s) {
  std::string host;
  int port = 0;
  if (!split_endpoint(endpoint, host, port)) return false;
  std::string resp;
  return http_post(host, port, body, resp, timeout_s) && parse_goals(resp, goals);
}

SubGoal VlmTaskPlanner::goal(uint32_t step, const std::string & skill, const std::string & target,
  const std::string & relation, float offset) {
  SubGoal g;
  g.step = step;
  g.skill = skill;
  g.target = target;
  g.relation = relation;
  g.offset_m = offset;
  return g;
}

}  // namespace plan
