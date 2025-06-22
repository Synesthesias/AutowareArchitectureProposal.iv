cmake_minimum_required(VERSION 3.14)
project(ndt_omp)

# Silence warnings about *_ROOT variables (CMP0144)
cmake_policy(SET CMP0144 NEW)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
add_compile_options(-Wall -Wextra)

if(BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native)
else()
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS common filters kdtree registration io visualization)
find_package(OpenMP)

# ====================
# Library: ndt_omp
# ====================
ament_auto_add_library(ndt_omp SHARED
  src/ndt_omp/voxel_grid_covariance_omp.cpp
  src/ndt_omp/ndt_omp.cpp
  # src/pclomp/gicp_omp.cpp
)

target_include_directories(ndt_omp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(ndt_omp PUBLIC ${PCL_DEFINITIONS})
target_link_directories(ndt_omp PUBLIC ${PCL_LIBRARY_DIRS})

target_link_libraries(ndt_omp ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(ndt_omp OpenMP::OpenMP_CXX)
endif()

# ====================
# Executable: align
# ====================
ament_auto_add_executable(align
  apps/align.cpp
)

target_link_libraries(align
  ${PCL_LIBRARIES}
  ndt_omp
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(align OpenMP::OpenMP_CXX)
endif()

# ====================
# Install
# ====================
install(
  TARGETS ndt_omp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ヘッダは include/ndt_omp 以下に配置されている前提
install(DIRECTORY include/
  DESTINATION include
)

# ====================
# Export
# ====================
ament_export_dependencies(PCL OpenMP)

# ====================
# Package
# ====================
ament_auto_package()
