cmake_minimum_required(VERSION 3.16)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# NOTE (Lyrical): ament_target_dependencies 매크로는 삭제됨 → target_link_libraries 사용.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TrackedObject.msg
  msg/TrackedObjectArray.msg
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs)
rosidl_get_typesupport_target(msgs_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)
include_directories(include)

# --- 노드: 클래스 구현(src/<노드>.cpp)은 라이브러리 하나로 묶고, 실행 파일(src/<노드>_main.cpp)과 테스트가 함께 링크
#     선언·설명은 include/perception/<노드>.hpp
add_library(${PROJECT_NAME}_nodes STATIC src/perception_node.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC rclcpp::rclcpp ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} ${msgs_cpp} diagnostic_updater::diagnostic_updater)
foreach(node perception_node)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_nodes)
endforeach()
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})

# 파라미터 (bringup이 share/perception/config/perception.yaml을 노드에 넘긴다)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# --- 테스트: colcon test --packages-select perception (노드 클래스를 같은 프로세스에 띄워 토픽으로 확인)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception_node test/test_perception_node.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=101)
  target_link_libraries(test_perception_node ${PROJECT_NAME}_nodes)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
