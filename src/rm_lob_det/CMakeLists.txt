cmake_minimum_required(VERSION 3.8)
project(detector)

# 1. ROS 2 Humble 标准要求 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 开启严格的警告检查，保证代码质量
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 导出编译命令供 clangd (VSCode) 提供完美的语法补全
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==========================================================
# 2. 查找依赖包 (显式查找，不依赖 auto 魔法)
# ==========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# ==========================================================
# 3. 编译动态链接库 (组件核心)
# ==========================================================
add_library(lob_vision_node SHARED
  src/lob_vision_node.cpp
  src/lob_tracker.cpp
  src/guide_light_detector.cpp
)

# 附加头文件目录
target_include_directories(lob_vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接 ROS 2 依赖与 OpenCV
ament_target_dependencies(lob_vision_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  rclcpp_components
  OpenCV
)

# ==========================================================
# 4. 注册为 ROS 2 组件 (生成独立的 _exec 启动程序)
# ==========================================================
rclcpp_components_register_node(lob_vision_node
  PLUGIN "rm_auto_aim::LobVisionNode"
  EXECUTABLE lob_vision_node_exec
)

# ==========================================================
# 5. 安装指令 (极其重要：告诉 ROS 2 去哪里找你的库和配置文件)
# ==========================================================
# 安装核心动态库
install(TARGETS lob_vision_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装 config 和 launch 文件夹
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 include 头文件目录
install(DIRECTORY include/
  DESTINATION include
)

# 结束构建
ament_package()