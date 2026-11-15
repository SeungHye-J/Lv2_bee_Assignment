cmake_minimum_required(VERSION 3.16)
project(perception)
# 빌드 종류를 안 주면(colcon build 그대로) 최적화 없이(-O0) 빌드됨 — 실기 젯슨에서 perception 상자 검출이 8배 느렸음 (11-13)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall> $<$<COMPILE_LANGUAGE:CXX>:-Wextra> $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>)  # nvcc에는 안 넘김
endif()

# NOTE (Lyrical): ament_target_dependencies 매크로가 삭제됨 → target_link_libraries 사용.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
# 인지 노드 C++판 (perception_node) — 영상·YOLO·AprilTag·바코드
# AprilTag: OpenCV 4.7+는 objdetect(본체) · 4.6 이하는 contrib aruco — 있는 쪽을 씀 (젯슨 4.8 · PC 도커 4.6)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d imgcodecs OPTIONAL_COMPONENTS objdetect aruco)  # imgcodecs = 테스트 사진 읽기
find_package(message_filters REQUIRED)
# ONNX Runtime: 기본은 ROS 패키지 onnxruntime_vendor (CPU) · GPU(CUDA)판이 따로 깔려 있으면 ONNXRUNTIME_ROOT로 그 폴더를 줌
#   (예: colcon build --cmake-args -DONNXRUNTIME_ROOT=/opt/onnxruntime-gpu — include/onnxruntime_cxx_api.h · lib/libonnxruntime.so)
set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "GPU ONNX Runtime 폴더 (비면 /opt/onnxruntime → onnxruntime_vendor)")
if(NOT ONNXRUNTIME_ROOT AND EXISTS "/opt/onnxruntime/lib/libonnxruntime.so")
  set(ONNXRUNTIME_ROOT "/opt/onnxruntime")  # 젯슨: 팀이 CUDA판 ONNX Runtime을 여기 깔아 둠 (yolo_detector_cpp와 같은 자리)
endif()
if(ONNXRUNTIME_ROOT)
  find_path(ORT_INC onnxruntime_cxx_api.h PATHS "${ONNXRUNTIME_ROOT}/include" "${ONNXRUNTIME_ROOT}/include/onnxruntime" NO_DEFAULT_PATH)
  find_library(ORT_LIB onnxruntime PATHS "${ONNXRUNTIME_ROOT}/lib" NO_DEFAULT_PATH)
  add_library(onnxruntime::onnxruntime SHARED IMPORTED)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES IMPORTED_LOCATION "${ORT_LIB}" INTERFACE_INCLUDE_DIRECTORIES "${ORT_INC}"
    INTERFACE_LINK_OPTIONS "LINKER:-rpath,${ONNXRUNTIME_ROOT}/lib")
  message(STATUS "ONNX Runtime: ${ORT_LIB}")
else()
  find_package(onnxruntime_vendor REQUIRED)
endif()
# 바코드 (zxing-cpp C++, apt libzxing-dev) — 없으면 바코드 없이 빌드 (진단에 '없음')
find_package(ZXing QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TrackedObject.msg
  msg/TrackedObjectArray.msg
  msg/MissionSignal.msg
  msg/HandTarget.msg
  msg/HandRelative.msg
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs diagnostic_msgs)
rosidl_get_typesupport_target(msgs_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)  # 같은 패키지 메시지를 C++ 노드가 씀

# --- C++ 노드: 클래스 구현(src/<노드>.cpp)은 라이브러리로 묶고, 실행 파일(src/<노드>_main.cpp)과 테스트가 함께 링크
#     선언·설명은 include/perception/<노드>.hpp
add_library(${PROJECT_NAME}_nodes STATIC src/body_filter.cpp src/body_filter_core.cpp src/signal_decoder.cpp src/static_esdf_slice.cpp
  src/depth_gate.cpp src/hand_relative.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${geometry_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater urdf::urdf tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs
  ament_index_cpp::ament_index_cpp ${nav_msgs_TARGETS} opencv_core opencv_imgproc ${msgs_cpp})
# body_filter 점 판정 GPU판 (11-16, body_filter_core.hpp) — nvcc가 있으면 src/body_filter_cuda.cu · 없으면(PC 도커 · CI) CPU판만
#   CPU판 -ffp-contract=off · GPU판 --fmad=false: 곱셈-덧셈을 합치지 않아 두 판 판정이 비트까지 같음
#   nvcc 찾기: PATH에 없으면 /usr/local/cuda/bin (젯슨 기본 자리)
if(NOT CMAKE_CUDA_COMPILER AND NOT DEFINED ENV{CUDACXX} AND EXISTS "/usr/local/cuda/bin/nvcc")
  set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
endif()
include(CheckLanguage)
check_language(CUDA)
option(PERCEPTION_USE_CUDA "body_filter GPU판 빌드 (nvcc 있을 때)" ON)
if(CMAKE_CUDA_COMPILER AND PERCEPTION_USE_CUDA)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 87)  # 젯슨 Orin (다른 GPU면 -DCMAKE_CUDA_ARCHITECTURES=…)
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  add_library(${PROJECT_NAME}_cuda STATIC src/body_filter_cuda.cu)
  target_include_directories(${PROJECT_NAME}_cuda PRIVATE include src)
  target_compile_options(${PROJECT_NAME}_cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
  set_target_properties(${PROJECT_NAME}_cuda PROPERTIES POSITION_INDEPENDENT_CODE ON CUDA_STANDARD 17)
  target_link_libraries(${PROJECT_NAME}_cuda PUBLIC CUDA::cudart)
  target_compile_definitions(${PROJECT_NAME}_nodes PRIVATE PERCEPTION_BODY_CUDA)
  target_link_libraries(${PROJECT_NAME}_nodes PUBLIC ${PROJECT_NAME}_cuda)
  message(STATUS "body_filter GPU판: nvcc ${CMAKE_CUDA_COMPILER_VERSION} · arch ${CMAKE_CUDA_ARCHITECTURES}")
else()
  message(STATUS "body_filter GPU판 없음 (nvcc 없음 또는 PERCEPTION_USE_CUDA=OFF) — CPU판만")
endif()
set_source_files_properties(src/body_filter_core.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
# static_esdf_slice: map_backend:=static_2d에서 nvblox ESDF slice 대역 (RL 주행 controller 입력)
# depth_gate: nvblox depth에서 가까운 픽셀(팔·폴대)을 0으로 — ros_system scan_cam_gate.py(pole_depth_gate)의 C++판 (CPU 6 → 2 %)
# hand_relative: 손 카메라 상대값 (scripts/hand_relative.py의 C++판, 11-16 — 파이썬은 CPU 몰리면 ~1 s 멈춤) · 계산 hand_geom.hpp
foreach(node body_filter signal_decoder static_esdf_slice depth_gate hand_relative)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_nodes)
endforeach()
install(TARGETS body_filter signal_decoder static_esdf_slice depth_gate hand_relative DESTINATION lib/${PROJECT_NAME})
# 컴포넌트 (11-16 메모리) — 같은 클래스를 컨테이너 한 프로세스에 올림 (bringup core_composition). 실행 파일은 그대로
find_package(rclcpp_components REQUIRED)
set_target_properties(${PROJECT_NAME}_nodes PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(${PROJECT_NAME}_components SHARED src/components.cpp)
target_link_libraries(${PROJECT_NAME}_components ${PROJECT_NAME}_nodes rclcpp_components::component)
rclcpp_components_register_nodes(${PROJECT_NAME}_components "perception::BodyFilter" "perception::SignalDecoderNode" "perception::DepthGate" "perception::StaticEsdfSlice")
install(TARGETS ${PROJECT_NAME}_components LIBRARY DESTINATION lib)

# YOLO 인지 노드 — C++판이 perception_node (bringup·launch가 이 이름으로 띄움, 11/11 젯슨 CPU 때문에 C++로)
#   계산 함수 perception_core · YOLO 실행 yolo_onnx · 노드 perception_node (설명은 include/perception/<이름>.hpp)
add_library(${PROJECT_NAME}_cpp STATIC src/perception_core.cpp src/yolo_onnx.cpp src/tag_cuda.cpp src/perception_node.cpp)
target_include_directories(${PROJECT_NAME}_cpp PUBLIC include)
target_compile_features(${PROJECT_NAME}_cpp PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_cpp PUBLIC rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${action_msgs_TARGETS} ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS} ${diagnostic_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros ament_index_cpp::ament_index_cpp
  message_filters::message_filters ${OpenCV_LIBS} onnxruntime::onnxruntime ${msgs_cpp})
# AprilTag GPU (cuAprilTags) — 젯슨: ~/isaac_ros_ws의 cuapriltags_vendor(isaac_ros_nitros release-5.0) + CUDA가 있으면 (start.sh가 그 setup을 source)
#   없으면(PC 도커 · CI) tag_cuda가 빈 판으로 빌드돼 OpenCV aruco 그대로 · 설치: documents/jetson-orin-nano/APRILTAG_GPU.md
find_package(CUDAToolkit QUIET)
ament_index_has_resource(HAS_CUAPRILTAGS cuapriltags cuapriltags_vendor)
if(HAS_CUAPRILTAGS AND CUDAToolkit_FOUND)
  ament_index_get_resource(CUAPRILTAGS_REL cuapriltags cuapriltags_vendor)
  ament_index_get_prefix_path(PREFIX_PATHS)
  foreach(p IN LISTS PREFIX_PATHS)
    if(NOT CUAPRILTAGS AND EXISTS "${p}/${CUAPRILTAGS_REL}/lib/libcuapriltags.a")
      set(CUAPRILTAGS "${p}/${CUAPRILTAGS_REL}")
    endif()
  endforeach()
endif()
if(CUAPRILTAGS)
  target_compile_definitions(${PROJECT_NAME}_cpp PRIVATE PERCEPTION_CUAPRILTAGS)
  target_include_directories(${PROJECT_NAME}_cpp PRIVATE "${CUAPRILTAGS}/cuapriltags")
  target_link_libraries(${PROJECT_NAME}_cpp PUBLIC "${CUAPRILTAGS}/lib/libcuapriltags.a" CUDA::cudart)
  message(STATUS "AprilTag GPU: cuAprilTags ${CUAPRILTAGS} (CUDA ${CUDAToolkit_VERSION})")
else()
  message(STATUS "AprilTag GPU 없음 (cuapriltags_vendor · CUDA) — OpenCV aruco만")
endif()
# YOLO TensorRT (11/15) — 젯슨: apt libnvinfer-dev(TensorRT 헤더·라이브러리) + CUDA가 있으면 yolo_onnx가 엔진으로 추론
#   없으면(PC 도커 · CI · 헤더 안 깐 젯슨) TensorRT 없이 빌드 → ONNX Runtime 그대로 (perception.yaml tensorrt 무시)
find_path(TENSORRT_INC NvInfer.h PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu)
find_library(TENSORRT_LIB nvinfer)
if(TENSORRT_INC AND TENSORRT_LIB AND CUDAToolkit_FOUND)
  target_compile_definitions(${PROJECT_NAME}_cpp PRIVATE PERCEPTION_TENSORRT)
  target_include_directories(${PROJECT_NAME}_cpp PRIVATE "${TENSORRT_INC}")
  target_link_libraries(${PROJECT_NAME}_cpp PUBLIC "${TENSORRT_LIB}" CUDA::cudart)
  message(STATUS "YOLO TensorRT: ${TENSORRT_LIB}")
else()
  message(STATUS "YOLO TensorRT 없음 (libnvinfer-dev · CUDA) — ONNX Runtime만")
endif()
if(ZXing_FOUND)
  target_link_libraries(${PROJECT_NAME}_cpp PUBLIC ZXing::ZXing)
  message(STATUS "바코드: zxing-cpp ${ZXing_VERSION}")
else()
  message(WARNING "zxing-cpp 없음 (apt install libzxing-dev) — perception_node가 바코드를 못 읽음")
endif()
add_executable(perception_node src/perception_node_main.cpp)
target_link_libraries(perception_node ${PROJECT_NAME}_cpp)
if(ONNXRUNTIME_ROOT)
  # GPU판 ONNX Runtime이 실행 때도 먼저 잡히게 — ROS setup이 LD_LIBRARY_PATH에 넣는 CPU판(onnxruntime_vendor)보다
  # 실행 파일에 적힌 경로(RPATH)가 앞서도록 (기본 RUNPATH는 LD_LIBRARY_PATH에 밀림)
  target_link_options(perception_node PRIVATE "LINKER:--disable-new-dtags")
endif()
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
# 예전 파이썬판 (scripts/perception_node.py) — 되돌릴 때: launch에서 executable을 perception_node_py로
install(PROGRAMS scripts/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_node_py)
# 손 카메라 상대값 예전 파이썬판 (scripts/hand_relative.py + 계산 모듈 hand_geom.py를 같은 폴더에) — 되돌릴 때: launch hand_relative_py:=true
install(PROGRAMS scripts/hand_relative.py DESTINATION lib/${PROJECT_NAME} RENAME hand_relative_py)
install(FILES scripts/hand_geom.py DESTINATION lib/${PROJECT_NAME})

# 파라미터 (bringup이 share/perception/config/perception.yaml을 노드에 넘긴다)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# 패키지 단독 실행용 (전체 로봇은 bringup)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# 학습 가중치 — 워크스페이스 공용 lv2_module6/models/yolo/*.pt · *.onnx 를 share에 설치
# (urdf와 같은 패턴: ROS 밖 train_yolo.py가 쓰는 파일을 패키지가 가져옴)
set(MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../models/yolo" CACHE PATH "lv2_module6/models/yolo")
file(GLOB MODEL_FILES "${MODEL_DIR}/*.pt" "${MODEL_DIR}/*.onnx")  # .onnx = 젯슨용 (ultralytics가 onnxruntime으로 읽음)
if(MODEL_FILES)
  install(FILES ${MODEL_FILES} DESTINATION share/${PROJECT_NAME}/model)
else()
  message(WARNING "YOLO 가중치 없음 (${MODEL_DIR}/*.pt · *.onnx) — 노드는 빈 목록 발행")
endif()

# --- 테스트: colcon test --packages-select perception
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_node test/test_node.py TIMEOUT 120 ENV ROS_DOMAIN_ID=101)
  ament_add_pytest_test(test_hand_relative test/test_hand_relative.py TIMEOUT 60)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_body_filter test/test_body_filter.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=108)
  target_link_libraries(test_body_filter ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_body_filter_cuda test/test_body_filter_cuda.cpp TIMEOUT 120)
  target_link_libraries(test_body_filter_cuda ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_static_esdf_slice test/test_static_esdf_slice.cpp TIMEOUT 60)
  target_link_libraries(test_static_esdf_slice ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_hand_geom test/test_hand_geom.cpp TIMEOUT 60)  # hand_geom.hpp — test_hand_relative.py와 같은 시나리오
  target_link_libraries(test_hand_geom ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_depth_gate test/test_depth_gate.cpp TIMEOUT 60)
  target_link_libraries(test_depth_gate ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_signal test/test_signal.cpp TIMEOUT 60)
  target_link_libraries(test_signal ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_perception_core test/test_perception_core.cpp TIMEOUT 60
    ENV PERCEPTION_TEST_DATA=${CMAKE_CURRENT_SOURCE_DIR}/test/data)
  target_link_libraries(test_perception_core ${PROJECT_NAME}_cpp)
  ament_add_gtest(test_yolo_trt_graph test/test_yolo_trt_graph.cpp TIMEOUT 180
    ENV PERCEPTION_TEST_DATA=${CMAKE_CURRENT_SOURCE_DIR}/test/data PERCEPTION_TEST_MODEL_DIR=${MODEL_DIR})
  target_link_libraries(test_yolo_trt_graph ${PROJECT_NAME}_cpp)
  ament_add_gtest(test_perception_runtime test/test_perception_runtime.cpp TIMEOUT 120
    ENV ROS_DOMAIN_ID=191 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  target_link_libraries(test_perception_runtime ${PROJECT_NAME}_cpp)
  if(ZXing_FOUND)
    ament_add_gtest(test_barcode_fallback test/test_barcode_fallback.cpp TIMEOUT 60
      ENV PERCEPTION_TEST_DATA=${CMAKE_CURRENT_SOURCE_DIR}/test/data)
    target_link_libraries(test_barcode_fallback ${PROJECT_NAME}_cpp)
  endif()
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/perception.launch.py" --show-args
    TIMEOUT 60)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
