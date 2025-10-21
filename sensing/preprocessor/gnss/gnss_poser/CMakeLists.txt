cmake_minimum_required(VERSION 3.14)
project(gnss_poser)

find_package(ament_cmake_auto REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GeographicLib REQUIRED geographiclib)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/gnss_poser_core.cpp
)

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GeographicLib_INCLUDE_DIRS}
)

if(GeographicLib_CFLAGS_OTHER)
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${GeographicLib_CFLAGS_OTHER})
endif()

target_link_libraries(${PROJECT_NAME}_core
  ${GeographicLib_LIBRARIES}
)

ament_auto_add_executable(${PROJECT_NAME}
  src/gnss_poser_node.cpp
)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)

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
