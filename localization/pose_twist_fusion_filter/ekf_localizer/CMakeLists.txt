cmake_minimum_required(VERSION 3.14)
project(ekf_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ekf_localizer.cpp
  src/kalman_filter/kalman_filter.cpp
  src/kalman_filter/time_delay_kalman_filter.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "EKFLocalizer"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(ekf_localizer-test test/test_ekf_localizer.test
    test/src/test_ekf_localizer.cpp
    src/ekf_localizer.cpp
    src/kalman_filter/kalman_filter.cpp
    src/kalman_filter/time_delay_kalman_filter.cpp)

  target_link_libraries(ekf_localizer-test ${catkin_LIBRARIES})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
