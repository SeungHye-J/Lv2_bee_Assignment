# coordinator → rl_arm_correction · rl_nav_correction: 지금 어느 skill의 어느 단계인지 (학습 보정 정책을 켜고 입력을 만드는 데 씀)
#   2026-11-03 배포 결정: 퍽에 닿는 구간·상자 앞 도킹만 시뮬과 같은 기준 동작 + 학습 보정. 그 밖(skill="")엔 보정 0.
#   단계·입력 정의는 시뮬 학습 환경과 같음 (isaac/lab place_env · pick_env · drive_env, 브랜치 feature/skill-envs)
std_msgs/Header header
uint32 run_id             # skill 한 번 실행마다 +1 — 바뀌면 처짐 적분·보정·이전 행동을 0부터 (시뮬 에피소드 시작과 같음)
string skill              # skills.yaml 이름: place_cell · load_slot · pick_standing_puck · unload_slot · dock_box · drive_straight … · "" = 보정 끔
uint8 phase               # 팔: 0 정지(위) · 1 내려가기 · 2 안정 · 3 쥐기/놓기 · 4 올리기 · 5 대기 (시뮬 _phase와 같음)
bool holding              # 팔: 목표를 붙들고 있음 → 처짐 적분이 돎 (움직이는 동안은 안 돎)
bool residual_live        # 팔: 정책이 보정을 바꿀 수 있음 (손을 열거나 닫기 전까지) · false면 마지막 보정 유지
float64[] q_above         # 팔: 내려가기 시작 자세 joint1..5 [rad]
float64[] q_down          # 팔: 내려간 자세 joint1..5 [rad]
float64 goal_x            # 주행: 목표 자세 (map) [m, m, rad]
float64 goal_y
float64 goal_yaw
