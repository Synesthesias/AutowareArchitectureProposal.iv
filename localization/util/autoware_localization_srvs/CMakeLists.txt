cmake_minimum_required(VERSION 3.14)
project(autoware_localization_srvs)

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PoseArray.srv"
  "srv/PoseWithCovarianceStamped.srv"
  DEPENDENCIES geometry_msgs
)

ament_auto_package()
