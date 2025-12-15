cmake_minimum_required(VERSION 3.10)
project(ndt_omp_modified VERSION 0.0.0)

# Use C++17 for modern PCL/ROS and log4cxx
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -mavx causes a lot of errors!!
add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# warn performance issue when not building in release mode
if (CMAKE_BUILD_TYPE)
  string(TOUPPER "${CMAKE_BUILD_TYPE}" uppercase_CMAKE_BUILD_TYPE)
  if (NOT uppercase_CMAKE_BUILD_TYPE STREQUAL "RELEASE")
    message(WARNING
      "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} may cause a serious performance degradation. Please configure with -DCMAKE_BUILD_TYPE=Release.")
  endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_omp
  CATKIN_DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ndt_omp
  src/ndt_omp/voxel_grid_covariance_omp.cpp
  src/ndt_omp/ndt_omp.cpp
)

add_executable(align
  apps/align.cpp
)
add_dependencies(align
  ndt_omp
)
# pcl 1.7 causes a segfault without -O2 (or -O3) flag
target_compile_options(ndt_omp PUBLIC
  $<$<CONFIG:Debug>:-O2 -g>
)
target_link_libraries(align
  ${catkin_LIBRARIES}
  ndt_omp
)

install(TARGETS ndt_omp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/ndt_omp/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
