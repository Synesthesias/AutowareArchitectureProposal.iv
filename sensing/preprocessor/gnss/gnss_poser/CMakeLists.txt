cmake_minimum_required(VERSION 3.14)
project(gnss_poser)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ament + 依存
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# GeographicLib 検出（Config が無い環境も想定）
find_package(GeographicLib QUIET)

# ヘッダ（自前ヘッダは include/gnss_poser/** に置く）
include_directories(include)

# 実行ファイル（core と node を同一ターゲットに）
ament_auto_add_executable(gnss_poser_node
  src/gnss_poser_core.cpp
  src/gnss_poser_node.cpp
)

# GeographicLib をリンク（※プレーンシグネチャで統一！）
if(TARGET GeographicLib::GeographicLib)
  target_link_libraries(gnss_poser_node GeographicLib::GeographicLib)
else()
  # Config が無い環境向けフォールバック（変数方式）
  find_library(GeographicLib_LIBRARIES NAMES Geographic)
  find_path(GeographicLib_INCLUDE_DIRS NAMES GeographicLib/Config.h)
  if(NOT GeographicLib_LIBRARIES OR NOT GeographicLib_INCLUDE_DIRS)
    message(FATAL_ERROR
      "GeographicLib not found. Install 'libgeographic-dev' (rosdep key: geographiclib).")
  endif()
  target_link_libraries(gnss_poser_node ${GeographicLib_LIBRARIES})
  target_include_directories(gnss_poser_node PRIVATE ${GeographicLib_INCLUDE_DIRS})
endif()

# インストール
install(TARGETS gnss_poser_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ヘッダの推奨配置（include/gnss_poser/**）で警告解消
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)