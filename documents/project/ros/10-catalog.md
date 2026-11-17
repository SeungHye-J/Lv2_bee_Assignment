[목차](README.md) · [◀ 직접작성·제공](09-nodes.md) · [학습 설계 ▶](11-learn.md)

## 노드 사전

`lv2_module6/ros2_ws/src`의 모든 직접 작성 실행 파일·컴포넌트·플러그인을 패키지별로 정리한 표입니다. 근거는 각 패키지의 `CMakeLists.txt`(`add_executable` · `rclcpp_components_register_nodes` · 플러그인 xml)와 `launch/*.launch.py`입니다. 제공(📦) 노드(Nav2·MoveIt·RealSense 등)는 [09-nodes.md](09-nodes.md)에 있습니다. 토픽·서비스·액션 전체 명세는 [API.md](../../API.md)입니다.

- 언어: 전 패키지가 ament_cmake입니다. `setup.py`(ament_python) 패키지는 없고, 파이썬 노드는 `scripts/`를 `install(PROGRAMS)`로 설치한 것뿐입니다.
- 형태: **노드** = 단독 프로세스 · **컴포넌트** = `rclcpp_components`로 컨테이너에 올릴 수 있음 · **플러그인** = 다른 노드가 pluginlib으로 로드.
- 전체 로봇은 `bringup/launch/bringup.launch.py`가 띄웁니다. 아래 "launch"가 `bringup`이면 그 파일이 `node(...)`나 `component(...)`로 직접 띄우거나(`IncludeLaunchDescription`으로 `ros_system`·`skills` launch를 거쳐) 띄운다는 뜻입니다.
- `core_composition:=true`(기본)이면 작은 C++ 노드는 컨테이너 `control_nodes` · `mission_nodes`에 올라갑니다(`COMPONENTS` 표, `bringup.launch.py`). 노드 이름·토픽·파라미터는 단독 실행과 같고, `false`면 노드마다 프로세스입니다.

### bringup (launch 전용 패키지 + 보조 노드 1개)

| 이름 | 언어 | 형태 | 역할 | 주요 토픽·액션 | launch |
| --- | --- | --- | --- | --- | --- |
| `cmd_vel_relay` | C++ (`src/cmd_vel_relay.cpp`) · 예전 Python `cmd_vel_relay_py` | 노드 | Nav2가 없는 팔 시험에서 twist_mux 출력을 바퀴로 그대로 전달 (스무딩·충돌 감시 없음) | 구독 `/cmd_vel_mux` → 발행 `/cmd_vel` (TwistStamped) | `stand_test.launch.py` (`cmd_vel_relay_py:=true`면 Python) |

launch 파일: `bringup.launch.py`(전체) · `low_memory.launch.py`(bringup을 저메모리 인자로 포함) · `stand_test.launch.py`(팔·서기 시험용, Nav2·SLAM 없이 ros_system + skills + 일부 노드).

### control

| 이름 | 언어 | 형태 | 역할 | 주요 토픽·액션 | launch |
| --- | --- | --- | --- | --- | --- |
| `arm_safety_monitor` | C++ | 노드 · 컴포넌트 `control::ArmSafetyMonitor` | 관절 전류로 접촉 감지, 궤적 취소 | `/joint_states` 구독 · `/arm_safety_event` 발행 · `/arm_controller/follow_joint_trajectory` cancel_goal 호출 | `bringup` (`control_nodes`), `control.launch.py` |
| `manual_control` | C++ | 노드 · 컴포넌트 `control::ManualControl` | 수동 모드 원격 조종, 팔·그리퍼 직접 명령, 모터 재부팅 중계 | 구독 `/cmd/manual/cmd_vel` · `/cmd/manual/arm` · `/cmd/manual/reboot_motors` · `/state/operation_mode` → 발행 `/cmd_vel_manual` · `/arm_controller/joint_trajectory`, 액션 클라 `/gripper_controller/gripper_cmd`, 서비스 `/beebotsystem/reboot_motors` · `/collision_monitor/toggle` | `bringup` (`control_nodes`), `control.launch.py`, `stand_test` |
| `BeebotSystemHardware` | C++ | ros2_control `SystemInterface` 플러그인 (`control_plugins.xml`, `src/beebot_system_hardware.cpp`) | OpenCR와 USB 시리얼(`C` 명령 · `S` 상태 · `E` 모터 에러), 관절 영점 offset, mock 모드 | `/motor_status`, `~/reboot_motors` ([09-nodes.md](09-nodes.md)) | URDF `<ros2_control>` 태그 → `controller_manager`(`ros_system.launch.py`의 `ros2_control_node`) |

### manager

| 이름 | 언어 | 형태 | 역할 | 주요 토픽·액션 | launch |
| --- | --- | --- | --- | --- | --- |
| `state_manager` | C++ | 노드 (컴포넌트 아님 — 재시작 담당이라 따로) | 로봇 상태의 주인. 조작·지도 모드, 장치 상태, 비상정지, 재시작(`restart.cpp`) | 발행 `/state/emergency` · `/state/collision` · `/state/devices` · `/state/devices/<이름>` · `/state/map_mode` · `/state/map_name` · `/state/map_list` · `/state/operation_mode` 등 (카메라·`/odom`·`/imu`·`/joint_states`를 구독해 장치 상태 판단) | `bringup`, `manager.launch.py` |

### plan

| 이름 | 언어 | 형태 | 역할 | 주요 토픽·액션 | launch |
| --- | --- | --- | --- | --- | --- |
| `coordinator` | C++ | 노드 · 컴포넌트 `plan::Coordinator` | 계획 단계를 skill 액션 서버에 맡기는 순서, 적재함 장부, 재시도 | 액션 서버 `/task/execute_plan` · 구독 `/task/sub_goals`(legacy) · `/tracked_objects` · `/state/operation_mode` · `/state/emergency` → 발행 `/task/status` · `/task/current_plan` · `/coordinator/basket`, 서비스 `/coordinator/basket_clear`, 액션 클라 `/skills/<skill>` | `bringup` (`mission_nodes`), `plan.launch.py`, `stand_test` |
| `mission_manager` | C++ | 노드 · 컴포넌트 `plan::MissionManager` | 주사위 미션 순서, 퍽 할당(`mission_solver.cpp`), 학습 플래너 `plan_next` 호출, 경기장 벽 지도 | 액션 클라 `/task/execute_plan` · `/cmd/mission` 구독 → 발행 `/mission/state` · `/mission/markers` · `/arena/walls` · `/arena/walls_only` · `/arena/correction` · `/localization/map_correction` · `/cmd/beep` | `bringup` (`mission_nodes`, `use_mission_manager`), `plan.launch.py` |

### skills (컴포넌트 10개, 컨테이너 2개)

`skills/launch/skills.launch.py`가 `rclcpp_components`의 `component_container`(비-mt) 두 개에 올립니다. 클래스는 `skills/CMakeLists.txt`의 `foreach(cls ...)`로 등록, 공통 베이스는 `src/skill_node.cpp`(액션 서버 `/skills/<skill>` + `models/skills/<skill>/` 정책 로드). 모두 C++, launch는 `skills.launch.py`(→ `bringup`이 `use_mission`일 때 포함, `stand_test`도 포함).

| 컴포넌트 (노드 이름) | 컨테이너 | 역할 | 주요 인터페이스 |
| --- | --- | --- | --- |
| `PoseSkill` (`skill_pose`) | `arm_skills` | 이름 붙은 팔 자세로 이동 (시작·스캔 자세 포함) | 발행 `/arm_controller/joint_trajectory` · 구독 `/state/emergency` · `/task/status` |
| `InspectSkill` (`skill_inspect`) | `arm_skills` | 상자·퍽 검사(보기) | 구독 `/tracked_objects`, 액션 클라 `/backup` · `/drive_on_heading` |
| `PickSkill` (`skill_pick`) | `arm_skills` | 서 있는 퍽 집기 | 서비스 클라 `/score_grasps` · `/apply_planning_scene` · `/clear_octomap` |
| `PickFarSkill` (`skill_pick_far`) | `arm_skills` | pick이 닿지 않는 먼 퍽 | `/score_grasps` |
| `PlaceSkill` (`skill_place`) | `arm_skills` | 주사위 칸에 놓기 | — |
| `LoadSkill` (`skill_load`) | `arm_skills` | 적재함 칸에 싣기 | — |
| `UnloadSkill` (`skill_unload`) | `arm_skills` | 적재함에서 꺼내기 | — |
| `StandSkill` (`skill_stand`) | `arm_skills` | 누운 퍽 세우기(+`stand_pick`) | 구독 `/hand_relative` · `/tracked_objects`, 발행 `/arm_controller/joint_trajectory` |
| `NavigateSkill` (`skill_navigate`) | `base_skills` | Nav2로 이동, 움직임 정답표 재생(`taught_routes`) | Nav2 액션 클라 (`/navigate_to_pose` · `/navigate_through_poses` · `/compute_path_to_pose` · `/follow_path` · `/spin` · `/backup` · `/drive_on_heading`) · 구독 `/arena/correction` |
| `ApproachSkill` (`skill_approach`) | `base_skills` | 상자 앞 도킹 | 발행 `/cmd_vel_nav` · 구독 `/odom` · `/imu` · `/tracked_objects` |

### perception

| 이름 | 언어 | 형태 | 역할 | 주요 토픽·액션 | launch |
| --- | --- | --- | --- | --- | --- |
| `perception_node` | C++ (ONNX Runtime · 선택 TensorRT) · 예전 Python `perception_node_py` | 노드 (무거워서 컴포넌트 아님) | YOLO 검출, 3D 위치, 두 카메라 융합 추적, AprilTag·바코드·상자 윗면 | 구독 두 카메라 color/depth/camera_info · `/cmd/mission` · `/mission/capture_dice` → 발행 `/tracked_objects` · `/mission/qr` · `/mission/dice_image` · `/perception/<카메라>/detections` · `/diagnostics` | `bringup` (`use_perception`), `perception.launch.py`, `stand_test` |
| `signal_decoder` | C++ | 노드 · 컴포넌트 `perception::SignalDecoderNode` | 신호등 12회 점멸 해석 | `/tracked_objects` → `/mission/signal` · `/mission/signal_status` | `bringup` (`control_nodes`) |
| `body_filter` | C++ (CUDA판 `body_filter_cuda.cu`) | 노드 · 컴포넌트 `perception::BodyFilter` | pole_front_cam 점구름에서 자기 몸 제거 | `/pole_front_cam/depth/color/points` → `…/points_filtered` · `…/points_no_floor` | `bringup` (`control_nodes`), `perception.launch.py` |
| `hand_relative` | C++ · 예전 Python `hand_relative_py` | 노드 | gripper_cam 검출 + 목표 → 손 기준 상대값 (RL 팔 정책 관측) | `/hand_relative/target` · `/perception/gripper_cam/detections` → `/hand_relative` | `bringup` (`use_hand_relative`, `hand_relative_py:=true`로 Python) |
| `depth_gate` | C++ (예전 Python `ros_system/scan_cam_gate`) | 노드 · 컴포넌트 `perception::DepthGate` | nvblox용 depth에서 카메라 0.5 m 안을 0으로 | `…/aligned_depth_to_color/image_raw` → `…/image_nvblox` | `ros_system.launch.py` (노드 이름 `pole_depth_gate` · `pole_back_depth_gate`) |
| `static_esdf_slice` | C++ | 노드 · 컴포넌트 `perception::StaticEsdfSlice` | `map_backend:=static_2d`일 때 nvblox 대신 ESDF slice | `/map` + `…/points_filtered` → `/nvblox_node/combined_esdf_pointcloud` | `ros_system.launch.py` (static_2d일 때만) |

### ros_system

| 이름 | 언어 | 형태 | 역할 | 주요 토픽·액션 | launch |
| --- | --- | --- | --- | --- | --- |
| `cuvslam_slam_cpp` (노드 이름 `cuvslam_slam`) | C++ + CUDA (`cuvslam_gpu.cu`) | 노드 (cuVSLAM 라이브러리가 있을 때만 빌드) | cuVSLAM 시각 SLAM | `/pole_front_cam/…` → `/cuvslam/map_to_odom` · `/cuvslam/odom` · `/cuvslam/path` · `/cuvslam/vis/*` | `ros_system.launch.py` (`cuvslam_impl:=auto\|cpp\|py`) |
| `cuvslam_slam` | Python (`scripts/cuvslam_slam.py`) | 노드 | 같은 역할의 Python판 (C++판이 없을 때 대체) | 위와 같음 | 같음 |
| `map_tf_relay` | C++ | 노드 | `/cuvslam/map_to_odom` → TF map→odom 30 Hz | — | `ros_system.launch.py` |
| `grid_obstacle_cloud` | C++ | 노드 | 도면 벽 `/arena/walls_only` → 점구름 `/arena/walls_cloud` (collision_monitor 입력) | — | `ros_system.launch.py` |
| `cpp_helpers` (인자 `nvblox_map` `rewind`) | C++ (`nvblox_map.cpp` + `rewind.cpp`를 한 프로세스) | 노드 | 아래 두 노드를 한 프로세스로 (`helpers_cpp` 기본) | — | `ros_system.launch.py` |
| `nvblox_map` | C++ (예전 Python `scripts/nvblox_map.py`) | 노드 | nvblox 격자 → `/map`(latched) + 저장 지도 `📞 /nvblox_node/load_map` | `/nvblox_node/static_occupancy_grid` · `/arena/walls` → `/map` | `ros_system.launch.py` |
| `rewind` | C++ (예전 Python `scripts/rewind.py`) | 노드 | 주행 ABORTED 뒤 궤적을 0.4 m 후진 | 서비스 `/rewind` · 구독 `/odometry/filtered` · `/navigate_*/_action/status` → `/cmd_vel_smoothed` | `ros_system.launch.py` |
| `py_helpers` | Python | 노드 (`rewind` · `nvblox_map` · `pole_depth_gate`를 한 프로세스에) | rclpy 노드 여럿을 `MultiThreadedExecutor` 하나로 (메모리 절약, `py_helpers_one_process`) | — | `ros_system.launch.py` (C++판이 아닐 때) |
| `scan_cam_gate` | Python | 노드 | `depth_gate`의 예전 Python판 | — | `ros_system.launch.py` |

그 밖에 `ros_system.launch.py`는 제공 노드(RealSense ×2, `robot_state_publisher`, `ros2_control_node` + spawner, `move_group`, `twist_mux`, `ekf_node`, Nav2 서버들, `nvblox_node`, `map_server`)를 띄우고 URDF를 설치합니다([09-nodes.md](09-nodes.md) 표 참고).

### ml

| 이름 | 언어 | 형태 | 역할 | 주요 인터페이스 | launch |
| --- | --- | --- | --- | --- | --- |
| `grasp_scorer` | C++ | 노드 · 컴포넌트 `ml::GraspScorer` | 잡기 후보 채점 (지금은 규칙 `rule_grasps`) | 서비스 서버 `/score_grasps` (`ml/srv/ScoreGrasps`) · 구독 `/tracked_objects` · `/gripper_cam/depth/color/points` | `bringup` (`mission_nodes`, `use_grasp_scorer`), `ml.launch.py` |
| `ml::RlNavController` (`rl_nav_controller`) | C++ | Nav2 controller 플러그인 (`rl_nav_controller.xml`) | ONNX 정책으로 경로 추종, inner(DWB·MPPI 등) 바이패스 | 서비스 `~/FollowPath/toggle` · `set_fallback` · `status`, `/diagnostics` | `controller_server`(`nav2_params.yaml`의 `FollowPath`) |
| `ml::RlArmCorrection` (`rl_arm_correction`) | C++ | ros2_control chainable controller 플러그인 (`rl_arm_correction.xml`) | arm_controller reference에 Δ(처짐 적분 + 정책 잔차 + 중력 앞먹임) | 구독 `/skill_context` (`ml/msg/SkillContext`), 발행 `~/debug` (`ml/ArmCorrectionDebug`) | `controllers.yaml` (`controller_manager` spawner) |

`ml/scripts/export_policy.py`는 노드가 아니라 TorchScript `.pt` → `.policy` 변환 도구입니다.

### faker (테스트베드 대역 — 실기 launch에는 없음)

`faker.launch.py`가 진짜 `bringup`을 포함하고, 하드웨어·카메라·ML·SLAM만 아래 대역으로 바꿉니다. 모두 C++.

| 이름 | 역할 | launch |
| --- | --- | --- |
| `fake_opencr` | 가상 OpenCR (가상 시리얼 + 충돌 월드 `faker/world/<코스>.yaml` + 몸 외곽 `robot_footprint.yaml`), `/fake_opencr/ground_truth` | `fake_hardware.launch.py` (이미 떠 있으면 `faker.launch.py`가 안 띄움) |
| `fake_camera` (이름 `pole_front_cam` · `gripper_cam`) | 파일/브리지 기반 카메라 토픽 | `faker.launch.py` (`use_gz_cameras:=false`) |
| `fake_cuvslam` | `/fake_opencr/ground_truth`로 map→odom 보정 | `faker.launch.py` (`fake_slam`) |
| `fake_nvblox` (이름 `nvblox_node`) | `/map`·ESDF (MAPPING은 점구름으로 지도를 쌓음, LOCALIZATION은 저장 지도 읽기) | `faker.launch.py` (`fake_slam`) |
| `fake_grasp_scorer` (이름 `grasp_scorer`) | `grasp_scorer` 대역 | `faker.launch.py` (`ml:=fake` 기본) |
| `faker/FakeRlArmCorrection` | `rl_arm_correction` 대역 컨트롤러 플러그인 (`faker_plugins.xml`) | `arm_correction_type` 인자 |
| `monitor_manager` | 토픽을 `<debug_dir>/topic/…`에 파일로 떨굼 (호스트 뷰어용) | `faker.launch.py` (`use_monitor`) |
| `input_manager` | 호스트가 쓴 `<debug_dir>/input/topic/…` 파일을 토픽으로 발행 | `faker.launch.py` |
| `service_action_monitor` | 서비스·액션 목록·정의를 파일로 | `faker.launch.py` |
| `service_action_bag_input` | 호스트 파일로 서비스 호출·액션 goal·rosbag 녹화 | `faker.launch.py` |

### Isaac 쪽 보조 노드 (ros2_ws 패키지 아님)

`isaac/standalone/runtime/`의 파이썬 노드는 [12-isaac.md](12-isaac.md)에 있습니다(`camera_adapter`, `arena_map_pub`, `node_params`, `cm_kick` 등).

### 확인 못 한 것

- 각 노드의 전체 토픽 목록은 소스의 `create_publisher`/`create_subscription` 문자열 검색으로만 뽑았습니다. 동적으로 이름을 만드는 토픽(예: `perception_node`의 카메라별 `/perception/<카메라>/…`)과 파라미터로 바뀌는 이름은 빠졌을 수 있습니다. 정확한 명세는 [API.md](../../API.md)가 기준입니다.
- `components.cpp`(control·perception·plan·ml)의 등록 클래스는 CMake 호출로만 확인했고 클래스 소스까지는 열지 않았습니다.
