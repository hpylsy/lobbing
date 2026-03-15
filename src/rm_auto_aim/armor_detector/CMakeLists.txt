# 1. 声明 CMake 最低版本和项目名称（必须放在最前面！）
cmake_minimum_required(VERSION 3.8)
project(armor_detector)

# 2. 编译选项设置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 3. 寻找 ROS 2 ament 依赖 (自动读取 package.xml)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 4. 寻找第三方视觉与 AI 库
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
set(OpenVINO_LIBRARIES openvino::runtime)

# 5. 寻找从源码编译的图优化与李代数库 (带上路径提示)
set(g2o_DIR "/usr/local/lib/cmake/g2o")
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)

# 6. 注册目标 (编译 src 目录下的源码)
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# ==========================================
# 7. 暴力指路区：绑定头文件和库文件路径
# ==========================================

# 【加回 PUBLIC】：这个指令在 CMake 里强制要求作用域关键字
target_include_directories(${PROJECT_NAME} PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenVINO_INCLUDE_DIRS} 
    "/opt/ros/humble/include"  
    "/usr/local/include"       
)


target_link_directories(${PROJECT_NAME} PUBLIC
    "/usr/local/lib"
    "/usr/local/lib/x86_64-linux-gnu"           
    "/opt/ros/humble/lib"
)

target_link_libraries(${PROJECT_NAME} 
    ${OpenVINO_LIBRARIES} 
    ${OpenCV_LIBS}
    g2o_core
    g2o_stuff
    g2o_solver_csparse
    g2o_types_sba
    g2o_types_slam3d
    g2o_solver_dense
)
# 8. 注册 ROS 2 组件节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

# 9. 测试与安装
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  models
  #launch
)