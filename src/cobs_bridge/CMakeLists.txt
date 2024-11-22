cmake_minimum_required(VERSION 3.8)
project(cobs_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cobs_bridge_interfaces REQUIRED)

add_executable(${PROJECT_NAME}
  src/cobs_bridge_node.cpp
  cobs-c/cobs.c
)
target_include_directories(${PROJECT_NAME} PRIVATE cobs-c)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  cobs_bridge_interfaces
)

add_executable(${PROJECT_NAME}_service
  src/cobs_bridge_service_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_service
  rclcpp
  cobs_bridge_interfaces
)

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_service
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
