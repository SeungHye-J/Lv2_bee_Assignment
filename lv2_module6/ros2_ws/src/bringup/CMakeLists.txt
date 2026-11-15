cmake_minimum_required(VERSION 3.16)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
# stand_test.launch.py 전용 — Nav2 없이 /cmd_vel_mux → /cmd_vel (수동 주행이 바퀴까지 가게)
#   C++판 src/cmd_vel_relay.cpp (11-16) · 예전 파이썬판은 cmd_vel_relay_py (launch cmd_vel_relay_py:=true)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(cmd_vel_relay src/cmd_vel_relay.cpp)
target_compile_features(cmd_vel_relay PRIVATE cxx_std_17)
target_link_libraries(cmd_vel_relay rclcpp::rclcpp ${geometry_msgs_TARGETS})
install(TARGETS cmd_vel_relay DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/cmd_vel_relay.py DESTINATION lib/${PROJECT_NAME} RENAME cmd_vel_relay_py)

# --- 테스트: colcon test --packages-select bringup
if(BUILD_TESTING)
  # launch 파일이 문법·인자 선언 오류 없이 읽히는지 (노드는 띄우지 않음 — 실행 흐름은 docker/lv2_module6/test-faker)
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/bringup.launch.py" --show-args
    TIMEOUT 60)
  # core_composition (11-16 메모리): 컨테이너로 묶어도 노드 이름·파라미터가 따로 띄울 때와 같은지
  ament_add_test(test_core_composition GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/test/test_core_composition.py"
    TIMEOUT 60)
endif()

ament_package()
