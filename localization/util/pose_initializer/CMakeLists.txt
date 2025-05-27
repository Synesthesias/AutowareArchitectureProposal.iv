cmake_minimum_required(VERSION 3.14)
project(pose_initializer)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/pose_initializer_node.cpp
  src/pose_initializer_core.cpp
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
