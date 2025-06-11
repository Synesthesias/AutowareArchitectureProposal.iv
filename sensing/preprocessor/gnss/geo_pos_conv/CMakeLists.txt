cmake_minimum_required(VERSION 3.14)
project(geo_pos_conv)

# ament_cmake_auto使用
find_package(ament_cmake_auto REQUIRED)
set(AMENT_CMAKE_INCLUDE_INSTALL_DIR include/${PROJECT_NAME})
ament_auto_find_build_dependencies()

# ヘッダのインクルードディレクトリ
include_directories(include)

# ライブラリとしてビルド（実行ファイルではない）
ament_auto_add_library(${PROJECT_NAME}
  SHARED
  src/geo_pos_conv.cpp
)

# ヘッダを適切な場所にインストール
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_auto_package()
