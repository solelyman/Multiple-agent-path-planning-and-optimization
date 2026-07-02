cmake_minimum_required(VERSION 3.8)
project(topology_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(topology_interfaces REQUIRED)
find_package(guidance_planner REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(prm_node src/prm_node.cpp)
ament_target_dependencies(prm_node rclcpp nav_msgs geometry_msgs visualization_msgs topology_interfaces guidance_planner)
target_include_directories(prm_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(prm_node guidance_planner::guidance_planner)

install(TARGETS
  prm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
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
