cmake_minimum_required(VERSION 3.14)
project(pose2twist)

# ビルド設定
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ヘッダーファイル
include_directories(include)

# 実行ファイル
ament_auto_add_executable(pose2twist
  src/pose2twist_node.cpp
  src/pose2twist_core.cpp
)

# インストール
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_auto_package()
