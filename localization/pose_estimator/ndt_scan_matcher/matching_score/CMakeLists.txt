cmake_minimum_required(VERSION 3.14)
project(matching_score)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/matching_score.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
