cmake_minimum_required(VERSION 3.14)
project(map_tf_generator)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/map_tf_generator.cpp
)

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_auto_add_executable(${PROJECT_NAME}
  src/map_tf_generator_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
