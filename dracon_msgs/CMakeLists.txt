cmake_minimum_required(VERSION 3.5)
project(dracon_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # <-- Add this line

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Job.msg"
  "msg/JobList.msg"
  "msg/JobCost.msg"
  "msg/JobCostTable.msg"
  "msg/JobAssignment.msg"
  "msg/JobAssignmentTable.msg"
  "msg/JobStatus.msg"
  DEPENDENCIES std_msgs
)

ament_package()