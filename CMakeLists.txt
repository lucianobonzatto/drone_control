cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(drone_test src/drone_test.cpp include/ros_client.cpp include/drone_control.cpp)
install(TARGETS drone_test DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies( drone_test 
  rclcpp 
  std_msgs
  geometry_msgs
  geographic_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(centralize src/centralize.cpp include/ros_client.cpp include/drone_control.cpp)
install(TARGETS centralize DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies( centralize 
  rclcpp 
  std_msgs
  geometry_msgs
  geographic_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(pose_test_0 src/pose_test_0.cpp include/ros_client.cpp include/drone_control.cpp)
install(TARGETS pose_test_0 DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies( pose_test_0 
  rclcpp 
  std_msgs
  geometry_msgs
  geographic_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(pose_test_1 src/pose_test_1.cpp include/ros_client.cpp include/drone_control.cpp)
install(TARGETS pose_test_1 DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies( pose_test_1
  rclcpp 
  std_msgs
  geometry_msgs
  geographic_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(pose_test_2 src/pose_test_2.cpp include/ros_client.cpp include/drone_control.cpp)
install(TARGETS pose_test_2 DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies( pose_test_2
  rclcpp 
  std_msgs
  geometry_msgs
  geographic_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(pose_test_3 src/pose_test_3.cpp include/ros_client.cpp include/drone_control.cpp)
install(TARGETS pose_test_3 DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies( pose_test_3
  rclcpp 
  std_msgs
  geometry_msgs
  geographic_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(pose_test_4 src/pose_test_4.cpp include/ros_client.cpp include/drone_control.cpp)
install(TARGETS pose_test_4 DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies( pose_test_4
  rclcpp 
  std_msgs
  geometry_msgs
  geographic_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
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
