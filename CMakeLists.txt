cmake_minimum_required(VERSION 3.8)
project(rviz_info_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)  # 追加

# include directories
include_directories(
  include
)

# ビルド対象を指定
add_executable(rviz_info_display
  src/rviz_info_display.cpp
)

# 必要なライブラリをリンク
ament_target_dependencies(rviz_info_display
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  rviz_2d_overlay_msgs  # 追加
)

# ノードのインストール先を指定
install(TARGETS
  rviz_info_display
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/
# )

# テストの設定
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
