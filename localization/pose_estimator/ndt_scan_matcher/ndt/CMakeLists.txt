cmake_minimum_required(VERSION 3.14)
project(ndt)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/base.cpp
  src/pcl_generic.cpp
  src/pcl_modified.cpp
  src/omp.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

install(DIRECTORY include/
  DESTINATION include
)

ament_auto_package()
