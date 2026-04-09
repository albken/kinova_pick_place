cmake_minimum_required(VERSION 3.8)
project(kinova_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(pcl_conversions REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)

find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED COMPONENTS
  common
  io
  filters
  segmentation
  search
  features
)

add_executable(kinova_pick_place src/kinova_pick_place.cpp)
ament_target_dependencies(kinova_pick_place
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_msgs
  shape_msgs
  std_srvs
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
)

add_executable(fake_cloud src/fake_cloud.cpp)
ament_target_dependencies(fake_cloud
 rclcpp 
 sensor_msgs
 tf2_ros 
 pcl_conversions
 moveit_ros_planning_interface
)

add_executable(color_cloud_detector src/color_cloud_detector.cpp)
ament_target_dependencies(color_cloud_detector
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  pcl_conversions
)

target_include_directories(color_cloud_detector PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_LIBS} 
)

target_compile_definitions(color_cloud_detector PRIVATE
  ${PCL_DEFINITIONS}
)

target_link_libraries(color_cloud_detector
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS} 
)

target_include_directories(kinova_pick_place PUBLIC include)

install(TARGETS kinova_pick_place color_cloud_detector fake_cloud
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/command_interpreter.py
    scripts/voice_recognition.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz config
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
