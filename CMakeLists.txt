cmake_minimum_required(VERSION 3.8)
project(upo_markers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(arco_marker src/arco_marker.cpp)
ament_target_dependencies(arco_marker rclcpp visualization_msgs)

add_executable(marker_node src/marker_node.cpp)
ament_target_dependencies(marker_node rclcpp visualization_msgs)

# add_executable(siar_model_viz src/siar_model_viz.cpp)
# ament_target_dependencies(siar_model_viz rclcpp visualization_msgs)

install(TARGETS
  arco_marker marker_node 
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY 
  Resource/
  DESTINATION  share)
  install(DIRECTORY 
  Resource/
  DESTINATION  share/upo_markers)

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
