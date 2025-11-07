cmake_minimum_required(VERSION 3.14)
project(autoware_localization_srvs)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(SRV_FILES
  "srv/PoseWithCovarianceStamped.srv"
  "srv/PoseArray.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

ament_auto_package()
