cmake_minimum_required(VERSION 3.14)
project(map_tf_generator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(map_tf_generator_node
  src/map_tf_generator.cpp
)

install(TARGETS map_tf_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
