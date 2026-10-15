cmake_minimum_required(VERSION 3.16)
project(ros_system)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# 로봇 모델은 워크스페이스 밖 lv2_module6/urdf 에 둔다 (Isaac Sim 등 ROS 밖에서도 같은 파일을 쓰므로).
# 테스트베드 컨테이너는 같은 상대 위치(/ws/src/ros_system/../../../urdf = /urdf)로 마운트한다.
set(URDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../urdf" CACHE PATH "lv2_module6/urdf")
if(NOT EXISTS "${URDF_DIR}/beebot.urdf.xacro")
  message(FATAL_ERROR "URDF 없음: ${URDF_DIR}/beebot.urdf.xacro (도커면 ../lv2_module6/urdf:/urdf 마운트 확인)")
endif()
install(DIRECTORY "${URDF_DIR}/" DESTINATION share/${PROJECT_NAME}/urdf)

# --- 테스트: colcon test --packages-select ros_system
if(BUILD_TESTING)
  # URDF: xacro로 펼쳐지고(mock 인자 포함) 링크·관절 트리가 올바른지
  ament_add_test(test_urdf GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND bash -c "xacro '${URDF_DIR}/beebot.urdf.xacro' mock_hardware:=true > '${CMAKE_CURRENT_BINARY_DIR}/beebot.urdf' && check_urdf '${CMAKE_CURRENT_BINARY_DIR}/beebot.urdf'"
    TIMEOUT 60)
endif()

ament_package()
