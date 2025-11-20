cmake_minimum_required(VERSION 3.14)
project(initial_pose_button_panel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/initial_pose_button_panel.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_NO_KEYWORDS)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/plugins
)

ament_auto_package()
