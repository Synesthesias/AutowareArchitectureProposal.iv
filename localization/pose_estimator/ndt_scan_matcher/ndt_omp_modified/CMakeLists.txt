cmake_minimum_required(VERSION 3.14)
project(ndt_omp_modified)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED COMPONENTS common filters io registration)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ndt_omp/voxel_grid_covariance_omp.cpp
  src/ndt_omp/ndt_omp.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC
    ${PCL_DEFINITIONS}
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    -msse
    -msse2
    -msse3
    -msse4
    -msse4.1
    -msse4.2
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

target_compile_options(${PROJECT_NAME} PUBLIC $<$<CONFIG:Debug>:-O2 -g>)

ament_auto_add_executable(align apps/align.cpp)
target_link_libraries(align ${PROJECT_NAME} ${PCL_LIBRARIES})

target_compile_options(align
  PRIVATE
    -msse
    -msse2
    -msse3
    -msse4
    -msse4.1
    -msse4.2
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(align OpenMP::OpenMP_CXX)
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_auto_package()
