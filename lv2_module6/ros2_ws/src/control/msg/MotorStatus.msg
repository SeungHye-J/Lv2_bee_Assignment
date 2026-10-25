# 모터(DYNAMIXEL)별 Hardware Error Status — BeebotSystemHardware가 펌웨어 E 줄(0.5초마다)을 받아 냄
#   /motor_status · 바뀔 때만 · reliable + transient_local (나중에 붙어도 마지막 값) → state_manager(/state/devices) 등
std_msgs/Header header
string[] joints   # 프로토콜 순서: wheel_left_joint, wheel_right_joint, joint1~5, gripper_left_joint
int32[] error     # 0 정상 · -1 응답 없음 · 비트 1 입력전압 · 4 과열 · 8 엔코더 · 16 전기충격 · 32 과부하 (Reboot로만 풀림)
string[] text     # 사람이 읽는 이름 — "정상" · "과부하" · "과열·과부하" · "응답 없음"
