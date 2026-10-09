cmake_minimum_required(VERSION 3.22)
project(multi_arm_motion_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionPrediction.msg"
  "action/ExecuteMotion.action"
  DEPENDENCIES trajectory_msgs builtin_interfaces moveit_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
