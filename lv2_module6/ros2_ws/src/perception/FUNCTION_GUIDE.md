# Perception 함수 해설

> 기준: `develop` 커밋 `48cc6b2b` (2026-11-17). 줄 번호는 그 커밋 기준이다. 코드가 바뀌면 줄 번호도 바뀐다.
> 여기 적은 것은 **코드에 실제로 있는 함수**만이다. 기본으로 꺼진 기능은 "기본 꺼짐"이라 적었다.
> 개요는 [README.md](README.md), 시험 방법은 [VALIDATION_GUIDE.md](VALIDATION_GUIDE.md).

## 0. 한눈에 보기

`main()`은 노드를 만들고 `rclcpp::spin()`에 넘긴다 (`src/perception_node_main.cpp`). 그다음은 **콜백과 타이머가 각자 시점에** 돈다. 함수를 차례로 한 번 부르는 구조가 아니다.

| 언제 | 함수 | 하는 일 |
|---|---|---|
| 컬러+Depth 쌍이 올 때 | `on_frame` | 최신 영상만 저장 |
| camera_info가 올 때 | `on_info` | fx·fy·cx·cy 저장 |
| 0.25초마다 | `on_infer` → `update_tracks` → `fuse` | YOLO·태그·깊이·상자 → 트랙 갱신 |
| 1/rate_hz(0.1초)마다 | `on_publish` | 트랙 → `/tracked_objects` |
| 0.5초마다 | `read_barcodes` | 바코드 읽기 → `/mission/qr` |
| 1초마다 | `on_diag` | 상태·처리 시간 → `/diagnostics` |

타이머 등록: `src/perception_node.cpp:424-427`.

## 1. perception_node (src/perception_node.cpp)

### 생성자 `PerceptionNode::PerceptionNode` — `:174-428`
- **역할**: 파라미터를 읽고, 모델을 올리고, 구독·발행·타이머를 만든다.
- **모델**: `model_path`를 찾아(`resolve_model_path`) `.onnx`만 `YoloOnnx`로 연다 (`:349-371`). TensorRT 엔진 경로는 `~/.cache/beebot/trt/<모델>_fp16.engine` (`:202-210`).
- **AprilTag 장치**: `tag_backend`가 `opencv`가 아니면 GPU(cuAprilTags)를 시도하고, 안 되면 OpenCV (`:318-327`).
- **구독**: 카메라 2대(`pole_front_cam`, `gripper_cam`)의 컬러·정렬 Depth를 `ApproximateTime`으로 묶는다 (시각 차 `sync_slop` 0.05초, `:401-415`). `/cmd/mission`을 받아 바코드 찾기 창을 연다 (`:390-400`).
- **주행 중 절전** (`navigation_throttle`, 코드 기본 꺼짐 · bringup이 켬): Nav2 상태·남은 거리를 구독한다 (`:217-230`).

### `on_frame` — `:431-441`
- **입력**: 한 카메라의 컬러·Depth 메시지 쌍
- **하는 일**: `frames_[cam]`에 최신 쌍만 넣는다. 변환(cv::Mat)은 여기서 안 하고 추론 때 한다 (`materialize_frame`, `:473-486`).

### `on_info` — `:443-447`
- camera_info의 `K`에서 fx·fy·cx·cy를 저장한다. 이게 없으면(`fx <= 0`) 추론하지 않는다 (`:512`).

### `take_frames` — `:507-520`
- 추론할 카메라를 고른다.
- 건너뛰는 경우: 영상이 없음, 같은 영상을 이미 처리함, 주행 중 절전으로 쉴 차례, 영상이 `max_frame_age_s`(2초)보다 오래됨.

### `on_infer` — `:522-548`
- **역할**: 0.25초마다, 고른 카메라마다 YOLO를 돌리고 `update_tracks`로 넘긴다.
- **계산**: `model_->predict(bgr, confidence, agnostic_nms)`. YOLO·전체 처리 시간을 진단용으로 잰다 (`prof_add`).
- 모델이 없으면 아무것도 안 한다 (`:523`) → 빈 목록만 발행.

### `update_tracks` — `:550-823` (가장 큰 함수)
한 카메라 한 프레임을 처리한다. 순서:

1. **TF**: 카메라 → `odom` 변환을 영상 시각으로 찾는다. 없으면 이 프레임은 버린다 (`:555-556`).
2. **AprilTag**: GPU면 화면 전체, OpenCV면 화면 전체 또는 YOLO 퍽·상자 네모 안(`tag_roi`)에서 찾는다 (`:559-575`).
3. **자기 몸**: 팔·차체 링크 TF로 화면 속 팔 위치를 구한다 (`:577-580`).
4. **YOLO 네모마다** (`:602-712`):
   - 퍽이면
     - 깊이 `box_depth_m`, 너무 가까워 깊이가 없으면 태그 solvePnP 거리 (`:618-623`)
     - 색 `puck_color` → `settle_puck_color` (`:624-631`)
     - 크기 판정 `judge_puck` (`:635-637`)
     - 자세 `posture_from_height` (`:640-648`). 모르면 같은 카메라가 잰 시작 상자 윗면을 받침으로 다시 본다.
     - 태그 번호 `tag_in_box`, 태그로 모양·방향 (`:658-667`)
   - 신호등이면 깊이를 박스 안 가까운 20 % 지점으로 잰다. 벽까지 거리가 섞이지 않게 하려는 것 (`:676-678`). 색은 모델 이름 또는 HSV로 정한다 (`:680`).
   - 화면 좌표 + 깊이 → 3D: `X=(u-cx)z/fx`, `Y=(v-cy)z/fy`, `Z=z` → `rot·p + trans` (`:682-683`).
   - 퍽은 표면 → 중심으로 보정 (`puck_center`, 태그 가운데가 3 cm 안이면 태그 중심, `:684-687`).
   - 자기 몸이면 버린다 (`is_self`, `:688-694`).
   - 측정 오차 공분산 `meas_cov_cam` (`:695`).
5. **상자 윗면** (`box_detect` 기본 켬): Depth에서 바닥 위 5~9 cm 평면을 찾는다 (`find_box_tops`, `:724-729`). 화면 가장자리에 잘리면 방향을 지우고 `cut` 표시 (`:730-735`).
6. **장소 태그(12~14)**: 태그가 올라간 상자를 `location_box`로 바꾼다. 윗면을 못 찾으면 태그 위치를 상자 위치로 쓴다 (`:741-791`).
7. **합치기**: 겹친 검출 합치기(`merge_overlap_dets`) → `fuse` → 가까운 같은 퍽 트랙 합치기(`merge_tracks`) → 2초 지난 트랙 지우기(`prune`) (`:792-803`).
8. **발행**: 이번 프레임 검출을 `/perception/<cam>/detections`로 (`:804-821`). 보는 구독자가 있으면 `publish_debug`로 그림 (`:822`).

### `publish_debug` — `:825-878`
- 회색 얇은 네모 = YOLO 전부, 빨간 글씨 = 버린 이유, 색 굵은 네모 = 받아들인 검출, 흰 네모 = AprilTag (`:827` 주석).
- `/perception/<cam>/debug_image`에 구독자가 있을 때만 그린다 → 평소 CPU를 안 쓴다.

### `box_rois` — `:880-913`
- 상자 깊이 찾기를 YOLO `box` 네모 안으로 좁힌다 (`box_roi_yolo`, 코드 기본 꺼짐 · yaml 켬).
- 상자 전용 모델(`box_model_path`)이 있으면 그걸 쓴다 (yaml 기본 없음).
- 이번 영상에 `box` 네모가 없으면 화면 전체를 본다.

### `on_capture_dice` / `publish_dice` — `:915-1011`
- `/mission/capture_dice` 요청이 오면, 장소 상자 윗면을 정사각형으로 펴서 `/mission/dice_image`로 낸다. 원본은 `_raw`로 낸다.

### `puck_yaw` — `:1038-1107`
- 누운 퍽은 윗면 점들의 긴 축 방향(주기 180°)을 쓴다.
- 서 있는 원기둥은 방향이 없다.
- 그 밖은 태그 자세 방향(주기 90°)을 쓴다. 집을 때 손 방향에 쓰인다.

### `fuse` — `:1109-1191`
- **역할**: 이번 프레임 검출을 기존 트랙에 1:1로 붙인다.
- **계산**:
  1. 같은 물체 후보(`same_object`) 중 `match_dist`(0.4 m) 안에 있는 쌍만 본다.
  2. 그 쌍들을 마할라노비스 거리가 작은 순서로 짝짓는다 (`:1118-1133`).
  3. 짝이 정해지면 칼만 갱신(`kf_update`)으로 위치·공분산을 고친다 (`:1139`).
- 신호등은 하나뿐인 물체라 거리 한도 없이 붙인다 (`SINGLE_CLASSES`, `:1123`).
- 태그 번호는 한 번 읽으면 남긴다 (`:1149`). 퍽 색은 "모름" 관측이 아는 색을 덮지 않는다 (`:1147`).
- 잘린 상자 관측은 위치·크기를 갱신하지 않는다 (`:1136`).
- 짝이 없는 검출은 새 트랙이 된다 (`:1174-1187`).

### `prune` — `:1193-1207`
- 2초(`TRACK_TIMEOUT_S`) 안 보인 트랙을 지운다.
- 장소 상자 자리의 `start_box`도 지운다.

### `map_transform` / `to_map` / `lookup_tf` — `:1222-1265`, `:1319-1351`
- 추적은 `odom`에서 하고, 발행 순간 `map ← odom`으로 바꾼다.
- map TF가 아직 한 번도 없으면 `odom` 좌표를 그대로 낸다 (`map_tf_required` 기본 false, `:1232-1235`).
- 영상 시각 TF가 없으면 최신 TF를 쓰되, 시각 차가 0.3초를 넘으면 그 관측을 버린다 (`:1330-1338`).

### `to_msg` — `:1267-1317`
- 트랙 → `TrackedObject`. 누운 퍽은 `puck_lying`으로 낸다 (`:1271`).
- `details`에 `posture`, `state`, `shape`, `yaw_period`, `distance`, `tag`, `content`를 넣는다. 상자는 `face_dist_m`, `cut`, `corners`, `faces`도 넣는다.

### `on_publish` — `:1354-1392`
- 0.1초마다(`rate_hz` 10) 모든 트랙을 `map` 좌표로 바꿔 `/tracked_objects`로 낸다.
- 로봇~물체 거리(`distance`)는 발행 순간 다시 잰다 (`:1385`).
- 트랙이 없으면 빈 목록을 낸다 (살아 있다는 신호).

### `read_barcodes` — `:1395-1457`
- **언제**: 미션 시작(`start`·`start:BARCODE`·`start:DIAG…`) 뒤 `barcode_window_s`(180초) 동안만 찾는다 (`:1410`). 항상 찾으려면 `barcode_always`를 켠다 (기본 꺼짐).
- **카메라**: 창을 연 뒤 3초(`barcode_pole_s`)는 폴 카메라도 쓰고, 그 뒤는 손 카메라만 쓴다. 폴 카메라는 신호등 YOLO에 양보한다 (`:1421-1424`).
- **읽기**: `zxing-cpp` 1D(Code128 등)를 쓰고, 실패하면 보조 읽기(`barcode_fallback`)를 한다 (`:117-148`). zxing 없이 빌드되면 바코드를 못 읽는다.
- **출력**: `/mission/qr`(원문), 그리고 `barcode` 트랙(`content`).
- `P2-0[1-4]` 형식이면 고정하고 더 안 찾는다 (`barcode_once`, `:1450`).

### `on_diag` — `:1459-1529`
- 1초마다 `/diagnostics`에 상태를 낸다.
  - 추적 물체 수, 두 카메라 융합 수, 동기화 프레임 수, 자기 몸으로 버린 수
  - YOLO 장치(`provider`), AprilTag 장치, 바코드 라이브러리 유무
  - 카메라별 `YOLO · AprilTag · 상자 · 전체 · 간격` 처리 시간 평균/최대 [ms]

## 2. YOLO 추론 (src/yolo_onnx.cpp)

### `YoloOnnx::YoloOnnx` — `:191-299`
- **순서**: TensorRT 엔진 → ONNX Runtime CUDA → CPU.
  - TensorRT: 엔진이 있고, `.onnx`보다 새것이고, 입출력 모양이 같을 때만 쓴다 (`:214-261`). 쓸 수 있으면 CUDA 그래프로 한 번에 띄운다 (`tensorrt_cuda_graph`, `:137-160`). 안 되면 `enqueueV3`.
  - TensorRT 없이 빌드됐으면 ONNX Runtime으로 간다 (`:261`).
  - ONNX Runtime에 `CUDAExecutionProvider`가 없으면 CPU로 간다 (`:268-276`).
- `provider()`가 실제로 쓰는 장치를, `error()`가 내려간 이유를 알려 준다 (`:307-308`).

### `YoloOnnx::predict` — `:311`~
- **전처리**: LetterBox(회색 114 여백) → BGR→RGB → /255 → NCHW.
- **후처리**: 클래스별 점수 → `conf` 이상만 → 클래스별 NMS(IoU 0.7, 최대 300) → 원본 좌표로 되돌림 (`include/perception/yolo_onnx.hpp` 머리말).
- **출력**: `YoloBox{x0,y0,x1,y1,conf,cls}` 목록.

## 3. 판단 함수 (src/perception_core.cpp)

| 함수 | 줄 | 역할 |
|---|---|---|
| `classify_color` | `:320` | 신호등 등 HSV 색 판정 |
| `puck_color` / `settle_puck_color` | `:359`, `:389` | 퍽 색 (빨강·초록), 모델 색과 다르면 버림 (`check_model_color`) |
| `split_class` / `puck_shape` / `model_posture` | `:432-460` | 모델 클래스 이름 → 종류·색·모양·자세 |
| `box_depth_m` | `:503` | 퍽 네모 안 깊이 (팔 화소 제외) |
| `posture_from_height` | `:519-559` | 퍽 화소의 높이 p90 = 윗면 높이. 받침(바닥 0 · 상자 윗면) + 서 있음 0.06 / 누움 0.03과 비교해 가까운 쪽. 차이가 작으면 "모름" |
| `detect_tags` / `detect_tags_in` | `:595`, `:671` | OpenCV aruco AprilTag 36h11 (감마·반 크기 영상 포함, 전체 또는 네모 안) |
| `tag_in_box` | `:697` | 퍽 네모 안 태그 번호 |
| `tag_pnp` / `tag_yaw` / `tag_center` | `:724-790` | 태그 네 모서리로 거리·방향·중심 |
| `find_box_tops` | `:812-910` | Depth 점을 바닥 위 높이(5~9 cm)로 골라 격자로 묶고, 최소 크기 이상이면 상자 윗면 (가운데·크기·방향) |
| `box_face_dist` / `box_corners` / `box_faces` | `:912-945` | 상자 도킹용: 로봇 쪽 면까지 거리, 모서리, 면 |
| `tag_on_box` / `box_cut` | `:946`, `:954` | 장소 태그가 윗면 안인지, 상자가 화면에 잘렸는지 |
| `judge_puck` | `:971-989` | 퍽 색·크기(긴 변 2.5~10 cm 등) 판정, 모르면 가로·세로 비율로 자세 |
| `same_object` / `merge_overlap_dets` / `merge_tracks` / `merge_single` | `:139-245` | 같은 물체 판단과 합치기 |
| `is_self` / `arm_pixels` / `box_has_arm` | `:252-296` | 로봇 자기 몸 검출 버리기 |
| `meas_cov_cam` / `kf_update` | `:297-313` | 측정 공분산(거리² 비례), 칼만 갱신 |

## 4. GPU AprilTag (src/tag_cuda.cpp)

- `CudaTagDetector::detect` (`:83`): cuAprilTags로 화면 전체를 한 번에 본다.
- cuAprilTags 없이 빌드되면 `available()`이 false다 (`:125-129`). 그러면 OpenCV를 쓴다.

## 5. 신호등 해독 — signal_decoder (src/signal_decoder.cpp)

- **입력**: `/tracked_objects`의 `traffic_light` 색. `red | yellow | green | off | all` 중 하나다.
- **`SignalDecoderNode::on_objects`** (`:160-184`):
  - 0.6초 안에 본 신호등 중 켜진 것을 고른다 (`light_state`).
  - 그 색을 `SignalDecoder::feed`에 넣는다.
  - 사이클이 끝날 때마다 `/mission/signal_status`로 낸다.
  - `ok`이고 새 답이면 `/mission/signal`로 낸다.
- **`feed`** (`:31`), **`boundary`** (`:84`), **`check`** (`:98`):
  - 켜진 구간 하나 = 점멸 1번이다. 구간 안에서 가장 많이 나온 색으로 정한다 (비율 < `min_share` 0.5면 unknown).
  - 0.3초(`min_off_s`)보다 짧은 꺼짐은 구간 안으로 본다. 0.3초(`min_on_s`)보다 짧은 켜짐은 잡음으로 버린다.
  - 꺼짐이 2.5초(`gap_s`) 이어지면 사이클 끝이다.
  - 검사: 12개인가(`count`), 모르는 색이 없나(`unknown`), 색마다 1~6회인가(`range`).
  - `confirm`(**1**)번 연속 같은 답이면 `ok=true`. 즉 지금 설정은 **12개를 한 번 다 보면 확정**이다 (`include/perception/signal_decoder.hpp:51`, `config/perception.yaml` `confirm: 1`).
  - 사이클 중간에 끊긴 뒷부분은 보관했다가, 다음 사이클 앞부분과 합쳐 12개가 되면 검사한다 (`tail_gap_max_s` 4.5초).
- 참고: `msg/MissionSignal.msg` 주석에는 "두 사이클 같게 읽힘 → ok"라고 남아 있다. 지금 설정(`confirm: 1`)과 다르다. 메시지 파일 주석은 이번 문서 작업에서 고치지 않았다 (코드 파일이라 별도 PR 대상).

## 6. 보조 노드

| 노드 | 파일 | 역할 | 입력 → 출력 |
|---|---|---|---|
| `body_filter` | `src/body_filter.cpp` (`on_cloud` `:150`) | URDF 충돌 모양을 TF로 카메라에 옮겨, 그 상자 안 점을 뺀다 | `/pole_front_cam/depth/color/points` → `…/points_filtered` (Nav2·충돌 감시), `…/points_no_floor` (MoveIt) |
| `hand_relative` | `src/hand_relative.cpp` (`on_dets` `:196`, `on_timer` `:146`) | 손 카메라 검출을 손(end effector) 기준 상대값으로 | `/hand_relative/target`, `/perception/gripper_cam/detections` → `/hand_relative` (팔 보정 정책 입력) |
| `pole_depth_gate` | `src/depth_gate.cpp` (`on_depth` `:43`) | 카메라 가까이(팔·폴대)의 깊이를 0으로 | depth → depth (nvblox 입력). 띄우는 곳은 `ros_system` launch (`depth_gate_cpp` 기본 true) |
| `static_esdf_slice` | `src/static_esdf_slice.cpp` | nvblox가 없을 때 2D 거리 지도 대역 | 정적 `/map` + 점구름 → `/nvblox_node/combined_esdf_pointcloud` |

- 컴포넌트 등록: `src/components.cpp`. bringup이 `core_composition`이면 `body_filter`·`signal_decoder`를 컨테이너에 올린다.
- 예전 파이썬판: `scripts/perception_node.py`(`perception_node_py`로 설치), `scripts/hand_relative.py`(`hand_relative_py`). 기본은 C++판이다.
