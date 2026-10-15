cmake_minimum_required(VERSION 3.16)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# --- 테스트: colcon test --packages-select bringup
if(BUILD_TESTING)
  # launch 파일이 문법·인자 선언 오류 없이 읽히는지 (노드는 띄우지 않음 — 실행 흐름은 docker/test-faker)
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/bringup.launch.py" --show-args
    TIMEOUT 60)
endif()

ament_package()
