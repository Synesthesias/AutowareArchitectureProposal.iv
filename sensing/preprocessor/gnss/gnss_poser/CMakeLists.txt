cmake_minimum_required(VERSION 3.14)
project(gnss_poser)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# GeographicLib 検出（Config.cmakeが無いため手動）
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h)
find_library(GeographicLib_LIBRARIES NAMES GeographicLib)

include_directories(
  include
  ${GeographicLib_INCLUDE_DIR}
)

ament_auto_add_executable(gnss_poser_node
  src/gnss_poser_core.cpp
  src/gnss_poser_node.cpp
)

target_link_libraries(gnss_poser_node
  ${GeographicLib_LIBRARIES}
)

install(TARGETS gnss_poser_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_auto_package()
