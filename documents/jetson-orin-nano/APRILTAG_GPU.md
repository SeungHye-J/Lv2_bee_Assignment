# AprilTag GPU (cuAprilTags) — perception_node CPU 줄이기

`perception_node`는 퍽 번호·장소 태그(tag36h11)를 찾는다. 예전에는 OpenCV aruco로 영상 한 장을 3번
(원본 · 감마 2.5 · 반 크기) 검사했고, 이게 젯슨에서 perception CPU의 대부분이었다.
지금은 **NVIDIA cuAprilTags(GPU)가 있으면 GPU로 한 번**, 없으면 예전 OpenCV 그대로 돈다.

- 코드: `lv2_module6/ros2_ws/src/perception/src/tag_cuda.cpp` (`include/perception/tag_cuda.hpp`)
- 파라미터: `tag_backend` — `auto`(기본: GPU판 빌드 + GPU 있으면 GPU) · `cuda` · `opencv`(예전 그대로)
- 확인 로그: `AprilTag: GPU (cuAprilTags)` 또는 `AprilTag: OpenCV aruco` · 진단 `/diagnostics` 인지 항목 "AprilTag"
- GPU가 한 장이라도 실패하면 그 장은 OpenCV로 (경고 10초에 한 번) — 멈추지 않음

> isaac_ros_apriltag **노드**는 쓰지 않는다. 그 노드는 rectify 영상과 별도 토픽(`/tag_detections`)이라
> YOLO 네모와 시각을 다시 맞춰야 한다. 같은 저장소(isaac_ros_nitros)의 **라이브러리(cuAprilTags)만** 가져와
> perception_node 안에서 부른다. Isaac ROS apt(release-5.0, noble)에는 apriltag 패키지가 없다 (2026-11-14 확인).

## 측정 (2026-11-14, Orin Nano 25W, JetPack 7.2 / CUDA 13.2)

| | OpenCV 3번 (예전) | GPU 한 번 |
|---|---|---|
| 실물 bag 392장 (perception-cpp-live · stand-test3, 두 카메라) 태그 | 125 | 125 (같은 번호 · 꼭짓점 차 평균 0.5 px, 최대 1.4 px) |
| CPU 시간 / 장 | 67.2 ms | 2.0 ms |
| 걸린 시간 / 장 (중앙 · p90 · 최대) | 29 · 34 · 42 ms | 3.3 · 3.8 · 4.3 ms |
| perception_node CPU (bag 재생, 100 % = 코어 하나) | 95 % (tag_roi 끔 · cv_threads 0) · 65 % (tag_roi · box_roi_yolo · cv_threads 2) | **30 %** |

test/data의 `glare_tag0`(광택 반사) · `big_tag7`(가까워 크게)도 GPU는 한 번에 찾는다 (OpenCV는 감마·반 크기 검사가 있어야 찾음).

## 설치 (젯슨 한 번)

cuAprilTags는 `isaac_ros_nitros`의 `cuapriltags_vendor`(정적 라이브러리 + 헤더)다. nvblox와 같은 `~/isaac_ros_ws`에 둔다.
배포 파일은 JetPack 6.1(CUDA 12.6)용뿐이지만 JetPack 7.2 / CUDA 13.2에서 링크·실행된다 (위 측정).

```bash
sudo apt-get install -y git-lfs
cd ~/isaac_ros_ws/src
git clone --depth 1 --filter=blob:none --sparse -b release-5.0 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
cd isaac_ros_nitros && git sparse-checkout set cuapriltags_vendor && git lfs install --local
git lfs pull --include="cuapriltags_vendor/lib_aarch64_jetpack61/*"
# release-5.0 CMakeLists가 설치하는 VERSION 파일이 저장소에 없음 (NVIDIA 쪽 누락) — 만들어 둠
echo "release-5.0 lib_aarch64_jetpack61" > cuapriltags_vendor/VERSION
cd ~/isaac_ros_ws && source /opt/ros/lyrical/setup.bash && colcon build --packages-select cuapriltags_vendor
```

그다음 ros2_ws의 perception을 다시 빌드한다 (`start.sh`가 `~/isaac_ros_ws/install/setup.bash`를 source하므로 `./start.sh --build`면 됨).
CMake 출력에 `AprilTag GPU: cuAprilTags ...`가 보이면 GPU판, `AprilTag GPU 없음`이면 OpenCV판이다.

```bash
source ~/isaac_ros_ws/install/setup.bash
colcon build --symlink-install --packages-select perception
```

PC 도커 · CI에는 cuapriltags_vendor가 없어 저절로 OpenCV판으로 빌드된다 (바꿀 것 없음).

## 꼭짓점 순서

cuAprilTags 꼭짓점은 aruco 순서를 두 칸 돌린 것이다 (`aruco[k] = cuAprilTags[(k + 2) % 4]`, 회전 30° · 120° · −100° · 200°로 확인).
`tag_cuda.cpp`가 aruco 순서로 바꿔 돌려주므로 `tag_pnp` · `tag_yaw` · `tag_center`는 그대로다.
회귀 시험: `test_perception_core`의 `Core.TagCudaSameAsOpenCV` (GPU 없으면 건너뜀).
