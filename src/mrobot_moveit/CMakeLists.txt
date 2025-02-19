cmake_minimum_required(VERSION 3.22)
project(mrobot_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(set_target_pose src/set_target_pose.cpp)
add_executable(visualizing_in_rviz src/visualizing_in_rviz.cpp)
add_executable(planning_around_objects src/planning_around_objects.cpp)

target_include_directories(set_target_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(visualizing_in_rviz PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_include_directories(planning_around_objects PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_compile_features(set_target_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(visualizing_in_rviz PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(planning_around_objects PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  set_target_pose
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

ament_target_dependencies(
  visualizing_in_rviz
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

ament_target_dependencies(
  planning_around_objects
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(TARGETS set_target_pose DESTINATION lib/${PROJECT_NAME})
install(TARGETS visualizing_in_rviz DESTINATION lib/${PROJECT_NAME})
install(TARGETS planning_around_objects DESTINATION lib/${PROJECT_NAME})

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