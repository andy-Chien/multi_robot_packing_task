cmake_minimum_required(VERSION 3.8)
project(task_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GQCNNGrasp.msg"
  "msg/BoundingBox.msg"
  "msg/Action.msg"
  "msg/Observation.msg"
  "msg/FlatVoxel.msg"
  "msg/RotationRange.msg"
  "srv/GQCNNGraspPlanner.srv"
  "srv/GQCNNGraspPlannerImg.srv"
  "srv/GQCNNGraspPlannerBoundingBox.srv"
  "srv/GQCNNGraspPlannerSegmask.srv"
  "srv/RoadmapSampleConfig.srv"
  "srv/GetPlannedTraj.srv"
  "srv/PackingPlanning.srv"
  "action/RoadmapSampler.action"
DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()