cmake_minimum_required(VERSION 3.16)
project(ros_system)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# map→odom TF 이어 내기 (src/map_tf_relay.cpp) — cuvslam_slam이 멈춰도 TF가 끊기지 않게
add_executable(map_tf_relay src/map_tf_relay.cpp)
target_compile_features(map_tf_relay PUBLIC cxx_std_17)
target_link_libraries(map_tf_relay PUBLIC rclcpp::rclcpp tf2_ros::tf2_ros ${geometry_msgs_TARGETS})
install(TARGETS map_tf_relay DESTINATION lib/${PROJECT_NAME})

# 보조 노드 C++판 (11-16) — nvblox_map · rewind를 한 프로세스(cpp_helpers <노드…>)로 · 파이썬판(py_helpers)과 이름·토픽 같음
#   include/ros_system/helpers.hpp · 격자 계산 grid_ops.hpp · launch helpers_cpp:=false면 예전 파이썬
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nvblox_msgs QUIET)  # nvblox_map 저장 지도 불러오기 (/nvblox_node/load_map) — ~/isaac_ros_ws를 source해야 잡힘
add_executable(cpp_helpers src/cpp_helpers.cpp src/nvblox_map.cpp src/rewind.cpp)
target_compile_features(cpp_helpers PRIVATE cxx_std_17)
target_include_directories(cpp_helpers PRIVATE include)
target_link_libraries(cpp_helpers rclcpp::rclcpp ${nav_msgs_TARGETS} ${std_srvs_TARGETS} ${action_msgs_TARGETS} ${geometry_msgs_TARGETS})
if(nvblox_msgs_FOUND)
  target_compile_definitions(cpp_helpers PRIVATE ROS_SYSTEM_NVBLOX_MSGS)
  target_link_libraries(cpp_helpers ${nvblox_msgs_TARGETS})
else()
  message(WARNING "nvblox_msgs 없음 — cpp_helpers nvblox_map이 저장 지도를 못 불러옴 (isaac_ros_ws source 후 다시 빌드)")
endif()
install(TARGETS cpp_helpers DESTINATION lib/${PROJECT_NAME})

# 도면 벽 → 로봇 주변 점구름 (src/grid_obstacle_cloud.cpp) — collision_monitor가 옆·뒤 벽을 보게
find_package(sensor_msgs REQUIRED)
add_executable(grid_obstacle_cloud src/grid_obstacle_cloud.cpp)
target_link_libraries(grid_obstacle_cloud rclcpp::rclcpp tf2_ros::tf2_ros ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS})
install(TARGETS grid_obstacle_cloud DESTINATION lib/${PROJECT_NAME})

# cuvslam_slam C++판 (src/cuvslam_slam.cpp, 11-16) — cuVSLAM C++ API(cuvslam2.h)를 바로 부름. 둘 다 있어야 빌드 (없으면 파이썬판만):
#   헤더 = PyCuVSLAM 소스(v17.0.1, NVIDIA Community License라 저장소에 안 넣음) CUVSLAM_SRC_DIR/libs/cuvslam/cuvslam2.h
#   라이브러리 = pip로 깐 cuvslam 패키지 안 libcuvslam.so (같은 소스로 빌드한 것 — documents/jetson-orin-nano/CUVSLAM.md)
set(CUVSLAM_SRC_DIR "$ENV{HOME}/cuvslam/src" CACHE PATH "PyCuVSLAM 소스 (cuvslam2.h)")
execute_process(COMMAND python3 -c "import cuvslam, os; print(os.path.dirname(cuvslam.__file__))"
  OUTPUT_VARIABLE CUVSLAM_PY_DIR OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
find_path(CUVSLAM_INCLUDE_DIR cuvslam2.h PATHS "${CUVSLAM_SRC_DIR}/libs/cuvslam" NO_DEFAULT_PATH)
find_library(CUVSLAM_LIB NAMES cuvslam PATHS "${CUVSLAM_PY_DIR}" NO_DEFAULT_PATH)
find_package(CUDAToolkit QUIET)
find_package(Eigen3 QUIET)
if(CUVSLAM_INCLUDE_DIR AND CUVSLAM_LIB AND CUDAToolkit_FOUND AND Eigen3_FOUND)
  find_package(message_filters REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  if(NOT CMAKE_CUDA_COMPILER)
    set(CMAKE_CUDA_COMPILER "${CUDAToolkit_NVCC_EXECUTABLE}")
  endif()
  set(CMAKE_CUDA_ARCHITECTURES 87)   # Orin
  enable_language(CUDA)              # src/cuvslam_gpu.cu — 가까운 depth 마스크 커널
  add_executable(cuvslam_slam_cpp src/cuvslam_slam.cpp src/cuvslam_gpu.cu)
  target_compile_features(cuvslam_slam_cpp PRIVATE cxx_std_17)
  target_include_directories(cuvslam_slam_cpp PRIVATE include src "${CUVSLAM_INCLUDE_DIR}")
  target_link_libraries(cuvslam_slam_cpp "${CUVSLAM_LIB}" CUDA::cudart Eigen3::Eigen rclcpp::rclcpp tf2_ros::tf2_ros
    message_filters::message_filters ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_msgs_TARGETS}
    ${visualization_msgs_TARGETS})
  # libcuvslam.so가 pip 패키지 폴더에 있어 실행할 때 찾게 (설치 위치가 바뀌면 다시 빌드)
  set_target_properties(cuvslam_slam_cpp PROPERTIES INSTALL_RPATH "${CUVSLAM_PY_DIR}" BUILD_RPATH "${CUVSLAM_PY_DIR}")
  install(TARGETS cuvslam_slam_cpp DESTINATION lib/${PROJECT_NAME})
  message(STATUS "cuvslam_slam_cpp: ${CUVSLAM_INCLUDE_DIR} · ${CUVSLAM_LIB}")
else()
  message(WARNING "cuvslam_slam_cpp 안 만듦 (cuvslam2.h · libcuvslam.so · CUDA · Eigen 중 없음) — launch cuvslam_impl:=py로")
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
# nvblox에 줄 depth에서 자기 몸(카메라 0.5 m 안)을 빼는 노드 (scripts/scan_cam_gate.py — 지금은 pole_front_cam만, gate_by_pose false)
install(PROGRAMS scripts/scan_cam_gate.py DESTINATION lib/${PROJECT_NAME} RENAME scan_cam_gate)
install(PROGRAMS scripts/cuvslam_slam.py DESTINATION lib/${PROJECT_NAME} RENAME cuvslam_slam)
# nvblox 격자 → /map(latched) + 저장 지도 불러오기 (scripts/nvblox_map.py 머리말)
install(PROGRAMS scripts/nvblox_map.py DESTINATION lib/${PROJECT_NAME} RENAME nvblox_map)
# 경로 리와인드 — 실패한 주행의 odom 궤적을 0.4 m 후진 (ros_system.launch.py가 띄운다)
install(PROGRAMS scripts/rewind.py DESTINATION lib/${PROJECT_NAME} RENAME rewind)
# 위 셋(rewind · scan_cam_gate · nvblox_map)을 한 프로세스로 (py_helpers_one_process) — py_helpers가 import하도록 .py도 같은 폴더에
install(PROGRAMS scripts/py_helpers.py DESTINATION lib/${PROJECT_NAME} RENAME py_helpers)
install(FILES scripts/rewind.py scripts/scan_cam_gate.py scripts/nvblox_map.py DESTINATION lib/${PROJECT_NAME})

# 로봇 모델은 워크스페이스 루트의 urdf/ 에 둔다 (Isaac Sim 등 ROS 밖에서도 같은 파일을 쓰므로).
# 모델 하나 = 폴더 하나: urdf/<이름>/urdf.xacro (+ meshes/) — launch 인자 urdf:=<이름>으로 고름 (기본 default)
# URDF = lv2_module6/urdf. 컨테이너는 /ws/urdf로 마운트(= src/../../urdf), 호스트 빌드는 lv2_module6/urdf(= src/../../../urdf)
# CACHE 아님 — 폴더를 바꾸면 다음 빌드에 바로 반영되게
set(URDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../urdf")
if(NOT IS_DIRECTORY "${URDF_DIR}")
  set(URDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../urdf")
endif()
file(GLOB URDF_XACROS CONFIGURE_DEPENDS "${URDF_DIR}/*/urdf.xacro")
if(NOT URDF_XACROS)
  message(FATAL_ERROR "URDF 없음: ${URDF_DIR}/<이름>/urdf.xacro (워크스페이스 루트의 urdf/ 확인)")
endif()
install(DIRECTORY "${URDF_DIR}/" DESTINATION share/${PROJECT_NAME}/urdf PATTERN "scripts" EXCLUDE)  # 생성·검수 스크립트는 설치 안 함

# --- 테스트: colcon test --packages-select ros_system
if(BUILD_TESTING)
  # cuvslam_slam C++판 계산 (slam_math.hpp) — test_map_gate.py와 같은 시나리오 (cuVSLAM·GPU 없이)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Eigen3 REQUIRED)
  ament_add_gtest(test_slam_math test/test_slam_math.cpp TIMEOUT 60)
  target_include_directories(test_slam_math PRIVATE include)
  target_link_libraries(test_slam_math Eigen3::Eigen)
  # nvblox_map C++판 격자 계산 (grid_ops.hpp — despeckle · overlay)
  ament_add_gtest(test_grid_ops test/test_grid_ops.cpp TIMEOUT 60)
  target_include_directories(test_grid_ops PRIVATE include)
  target_link_libraries(test_grid_ops ${nav_msgs_TARGETS})
  # URDF: 모델 폴더마다 xacro로 펼쳐지고(mock 인자 포함) 링크·관절 트리가 올바른지
  foreach(X ${URDF_XACROS})
    get_filename_component(D "${X}" DIRECTORY)
    get_filename_component(N "${D}" NAME)
    ament_add_test(test_urdf_${N} GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND bash -c "xacro '${X}' mock_hardware:=true > '${CMAKE_CURRENT_BINARY_DIR}/${N}.urdf' && check_urdf '${CMAKE_CURRENT_BINARY_DIR}/${N}.urdf'"
      TIMEOUT 60)
  endforeach()
  # SLAM(cuVSLAM + nvblox) 노드·지도 파일 경로가 맞게 펼쳐지는지 — GPU 없이 launch 구조만 (test/test_slam_launch.py)
  # cuVSLAM map→odom 거르기(MapGate) — 회전 중·큰 점프 유지, 안정된 새 값은 받아들임 (test/test_map_gate.py)
  ament_add_test(test_map_gate GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/test/test_map_gate.py")
  ament_add_test(test_slam_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/test/test_slam_launch.py"
    TIMEOUT 60)
  ament_add_test(test_static_map_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/test/test_static_map_launch.py"
    TIMEOUT 60)
  ament_add_test(test_cuvslam_image GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/test/test_cuvslam_image.py"
    TIMEOUT 60)
  # resource_watch는 ROS 패키지 밖(lv2_module6/tools)의 순수 Python 도구라
  # colcon test 대상이 아니다. 테스트는 tools/README-resource-watch.md의 수동 명령으로 실행한다.
endif()

ament_package()
