cmake_minimum_required(VERSION 3.14)
project(ekf_localizer)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

set(PACKAGE_LIB ${PROJECT_NAME}_component)

ament_auto_add_library(${PACKAGE_LIB} SHARED
  src/ekf_localizer_component.cpp
  src/kalman_filter/kalman_filter.cpp
  src/kalman_filter/time_delay_kalman_filter.cpp
)

target_include_directories(${PACKAGE_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(${PACKAGE_LIB} PUBLIC cxx_std_17)
target_link_libraries(${PACKAGE_LIB} Eigen3::Eigen)

rclcpp_components_register_nodes(${PACKAGE_LIB} "ekf_localizer::EKFLocalizerComponent")

ament_auto_add_executable(${PROJECT_NAME}
  src/ekf_localizer_node.cpp
)

target_link_libraries(${PROJECT_NAME} ${PACKAGE_LIB})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
