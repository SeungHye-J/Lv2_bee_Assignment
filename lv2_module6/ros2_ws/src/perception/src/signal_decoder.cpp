// signal_decoder 구현 — 노드 설명·입출력·멤버 선언은 include/perception/signal_decoder.hpp, main은 signal_decoder_main.cpp
#include "perception/signal_decoder.hpp"

#include <algorithm>

namespace perception {

namespace {
bool is_color(const std::string & s) { return s == "red" || s == "yellow" || s == "green"; }
bool is_off(const std::string & s) { return s == "off" || s.empty(); }
char letter(const std::string & c) { return c == "red" ? 'R' : c == "yellow" ? 'Y' : c == "green" ? 'G' : '?'; }
std::string text(const std::vector<std::string> & seg) {
  std::string t;
  for (const auto & c : seg) t += std::string(t.empty() ? "" : " ") + letter(c);
  return t;
}
}  // namespace

SignalDecoder::SignalDecoder(SignalParams p) : p_(std::move(p)) {}

bool SignalDecoder::feed(double t, const std::string & state, MissionSignal & out) {
  const std::string s = state.empty() ? "off" : state;
  if (since_ < 0) since_ = t;  // 첫 샘플 — 노드가 켜진 순간부터 잼 (켜기 전 시간을 꺼짐으로 세지 않음)
  if (s != cur_) {
    end_run(t);
    cur_ = s;
    since_ = t;
    marked_ = false;
  }
  // 사이클 끝: 꺼짐이 gap_s 넘게 이어짐 (꺼짐 구간마다 한 번)
  if (!marked_ && is_off(cur_) && t - since_ >= p_.gap_s) {
    marked_ = true;
    return boundary(out);
  }
  return false;
}

void SignalDecoder::end_run(double t) {
  // 끝난 켜짐 구간 하나 = 점멸 1번 (min_on_s보다 짧으면 잡음). 빨·노·초가 아닌 켜짐(all 등)은 추정하지 않고 unknown
  if (is_off(cur_) || t - since_ < p_.min_on_s) return;
  seg_.push_back(is_color(cur_) ? cur_ : "unknown");
}

bool SignalDecoder::boundary(MissionSignal & out) {
  if (!started_) {  // 첫 사이클 끝 — 그 전 조각은 사이클 중간부터 봤을 수 있어 버림
    started_ = true;
    seg_.clear();
    return false;
  }
  if (seg_.empty()) return false;  // 점멸 없이 꺼짐만 이어짐 (신호 시작 전 · 신호등 안 보임)
  check(out);
  seg_.clear();
  return true;
}

void SignalDecoder::check(MissionSignal & out) {
  out.sequence = seg_;
  out.red = static_cast<int32_t>(std::count(seg_.begin(), seg_.end(), "red"));
  out.yellow = static_cast<int32_t>(std::count(seg_.begin(), seg_.end(), "yellow"));
  out.green = static_cast<int32_t>(std::count(seg_.begin(), seg_.end(), "green"));
  out.raw = text(seg_);
  // 검사 순서: 모르는 색 → 개수 → 색별 범위 (앞에서 걸리면 그 이유 하나만)
  const auto out_of_range = [&](int32_t n) { return n < p_.min_count || n > p_.max_count; };
  if (std::count(seg_.begin(), seg_.end(), "unknown") > 0) out.error = "unknown";
  else if (static_cast<int>(seg_.size()) != p_.blinks) out.error = "count";
  else if (out_of_range(out.red) || out_of_range(out.yellow) || out_of_range(out.green)) out.error = "range";
  else out.error = "";
  if (!out.error.empty()) {  // 정상 사이클이 아니면 연속 횟수를 끊음 — 다음 정상 사이클부터 다시 셈
    same_ = 0;
    last_raw_.clear();
    out.cycles = 0;
    out.ok = false;
    return;
  }
  same_ = out.raw == last_raw_ ? same_ + 1 : 1;
  last_raw_ = out.raw;
  out.cycles = same_;
  out.ok = same_ >= p_.confirm;
  if (!out.ok) out.error = "unconfirmed";
}

std::string SignalDecoder::progress() const {
  return (started_ ? "" : "(첫 사이클 끝 기다리는 중) ") + text(seg_) +
         (is_color(cur_) ? " (" + std::string(1, letter(cur_)) + ")" : "") +
         " · " + std::to_string(seg_.size()) + "/" + std::to_string(p_.blinks);
}

SignalDecoderNode::SignalDecoderNode()
: rclcpp::Node("signal_decoder") {
  SignalParams p;
  p.min_on_s = declare_parameter<double>("min_on_s", p.min_on_s);
  p.gap_s = declare_parameter<double>("gap_s", p.gap_s);
  p.blinks = static_cast<int>(declare_parameter<int64_t>("blinks", p.blinks));
  p.min_count = static_cast<int>(declare_parameter<int64_t>("min_count", p.min_count));
  p.max_count = static_cast<int>(declare_parameter<int64_t>("max_count", p.max_count));
  p.confirm = static_cast<int>(declare_parameter<int64_t>("confirm", p.confirm));
  dec_ = SignalDecoder(p);
  pub_ = create_publisher<MissionSignal>("/mission/signal", rclcpp::QoS(1).reliable().transient_local());
  sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { on_objects(*m); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
              !have_ ? "신호 읽는 중" : last_.ok ? "확정: " + last_.raw : "마지막 사이클 " + last_.error + ": " + last_.raw);
    s.add("신호등", state_);
    s.add("지금 사이클", dec_.progress());
    if (have_) {
      s.add("마지막 결과", "R" + std::to_string(last_.red) + " Y" + std::to_string(last_.yellow) + " G" +
                          std::to_string(last_.green) + " · " + std::to_string(last_.cycles) + "사이클 연속");
    }
  });
}

void SignalDecoderNode::on_objects(const perception::msg::TrackedObjectArray & m) {
  // 가장 최근에 본 신호등 하나 — 0.6초 넘게 못 봤으면 '안 보임'(= 꺼짐으로 셈. 꺼진 신호등은 YOLO가 놓치기 쉬움)
  const rclcpp::Time now_t = now();
  const perception::msg::TrackedObject * best = nullptr;
  for (const auto & o : m.objects) {
    if (o.class_name != "traffic_light") continue;
    if (!best || rclcpp::Time(o.last_seen) > rclcpp::Time(best->last_seen)) best = &o;
  }
  state_ = best && (now_t - rclcpp::Time(best->last_seen, now_t.get_clock_type())).seconds() < 0.6 ? best->color : "";
  MissionSignal out;
  if (!dec_.feed(now_t.seconds(), state_, out)) return;
  out.header.stamp = now_t;  // 사이클 끝을 확인한 시각 (로그용)
  last_ = out;
  have_ = true;
  if (!out.ok) {  // 확정 전·오류 사이클 — 발행하지 않고 로그·진단에만 (planner가 받을 때마다 다시 계획하므로)
    RCLCPP_WARN(get_logger(), "신호 사이클 %s [%s] (%zu개)", out.error.c_str(), out.raw.c_str(), out.sequence.size());
    return;
  }
  if (out.raw == published_raw_) return;  // 반복 신호 — 같은 확정 답은 한 번만 발행 (사이에 오류 사이클이 껴도)
  published_raw_ = out.raw;
  pub_->publish(out);
  RCLCPP_WARN(get_logger(), "신호 확정 [%s] → R%d Y%d G%d (%d사이클 연속)",
    out.raw.c_str(), out.red, out.yellow, out.green, out.cycles);
}

}  // namespace perception
