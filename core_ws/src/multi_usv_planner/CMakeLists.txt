cmake_minimum_required(VERSION 3.8)
project(multi_usv_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(topology_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(g2o REQUIRED)

# 确保 g2o 头文件可被找到
get_target_property(G2O_INCLUDE_DIRS g2o::core INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "g2o include dirs: ${G2O_INCLUDE_DIRS}")

# ── Static Library ─────────────────────────────────
file(GLOB_RECURSE CONTOURING_ACADOS_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/acados/generated/contouring_solver/*.c"
)
list(FILTER CONTOURING_ACADOS_SOURCES EXCLUDE REGEX ".*/main(_sim)?_contouring_unicycle\\.c$")

add_library(${PROJECT_NAME} SHARED
  src/mpc_solver.cpp
  src/acados_solver.cpp
  src/acados_contouring_solver.cpp
  src/colregs_engine.cpp
  src/constraint_builder.cpp
  acados/generated/acados_solver_heron_single.c
  acados/generated/heron_single_model/heron_single_expl_ode_fun.c
  acados/generated/heron_single_model/heron_single_expl_vde_forw.c
  acados/generated/heron_single_model/heron_single_expl_vde_adj.c
  ${CONTOURING_ACADOS_SOURCES}
  src/controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/acados/generated>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/acados/generated/heron_single_model>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/acados/generated/contouring_solver>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/acados/generated/contouring_solver/contouring_unicycle_model>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/acados/generated/contouring_solver/contouring_unicycle_constraints>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE
  /home/lu/.local/share/acados/include
  /home/lu/.local/share/acados/include/acados
  /home/lu/.local/share/acados/include/blasfeo/include
  /home/lu/.local/share/acados/include/hpipm/include
  ${DECOMP_UTIL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  topology_interfaces
  tf2
  tf2_geometry_msgs
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  /home/lu/.local/share/acados/lib/libacados.so
  /home/lu/.local/share/acados/lib/libhpipm.so
  /home/lu/.local/share/acados/lib/libblasfeo.so
)

# ── Executable (Node) ──────────────────────────────
add_executable(usv_planner_node_exe src/usv_planner_node.cpp)
target_compile_features(usv_planner_node_exe PUBLIC cxx_std_17)
target_include_directories(usv_planner_node_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${DECOMP_UTIL_INCLUDE_DIRS}
)
ament_target_dependencies(usv_planner_node_exe
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  topology_interfaces
  tf2
  tf2_geometry_msgs
)
target_link_libraries(usv_planner_node_exe
  ${PROJECT_NAME}
  Eigen3::Eigen
)

add_executable(scene_visualizer_node src/scene_visualizer_node.cpp)
add_executable(heron_single_node src/heron_single_node.cpp)
add_executable(topology_selector_node src/topology_selector_node.cpp)
target_compile_features(scene_visualizer_node PUBLIC cxx_std_17)
target_compile_features(heron_single_node PUBLIC cxx_std_17)
target_compile_features(topology_selector_node PUBLIC cxx_std_17)
target_include_directories(scene_visualizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(heron_single_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(topology_selector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(scene_visualizer_node
  rclcpp
  geometry_msgs
  visualization_msgs
)
ament_target_dependencies(heron_single_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)
ament_target_dependencies(topology_selector_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  topology_interfaces
  tf2
)
target_link_libraries(heron_single_node
  ${PROJECT_NAME}
  Eigen3::Eigen
)
target_link_libraries(topology_selector_node
  ${PROJECT_NAME}
  Eigen3::Eigen
)

install(TARGETS ${PROJECT_NAME} usv_planner_node_exe topology_selector_node scene_visualizer_node heron_single_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  multi_usv_planner/dynamic_obstacle_node.py
  multi_usv_planner/floating_debris_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
