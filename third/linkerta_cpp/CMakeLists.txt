cmake_minimum_required(VERSION 3.10)
project(linkerta)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找ROS2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # 新增：添加sensor_msgs依赖

# 获取ROS2安装路径
if(DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO $ENV{ROS_DISTRO})
else()
    set(ROS_DISTRO "humble")
endif()

# 设置ROS2头文件路径
set(ROS2_INCLUDE_DIRS
    "/opt/ros/${ROS_DISTRO}/include"
    "/opt/ros/${ROS_DISTRO}/include/rclcpp"
    "/opt/ros/${ROS_DISTRO}/include/rcl"
    "/opt/ros/${ROS_DISTRO}/include/rcutils"
    "/opt/ros/${ROS_DISTRO}/include/rmw"
    "/opt/ros/${ROS_DISTRO}/include/rcl_yaml_param_parser"
    "/opt/ros/${ROS_DISTRO}/include/std_msgs"
    "/opt/ros/${ROS_DISTRO}/include/ament_index_cpp"
    "/opt/ros/${ROS_DISTRO}/include/builtin_interfaces"
    "/opt/ros/${ROS_DISTRO}/include/class_loader"
    "/opt/ros/${ROS_DISTRO}/include/composition_interfaces"
    "/opt/ros/${ROS_DISTRO}/include/libstatistics_collector"
    "/opt/ros/${ROS_DISTRO}/include/rclcpp_components"
    "/opt/ros/${ROS_DISTRO}/include/rclcpp_lifecycle"
    "/opt/ros/${ROS_DISTRO}/include/rcpputils"
    "/opt/ros/${ROS_DISTRO}/include/rosidl_runtime_c"
    "/opt/ros/${ROS_DISTRO}/include/rosidl_runtime_cpp"
    "/opt/ros/${ROS_DISTRO}/include/rosidl_typesupport_c"
    "/opt/ros/${ROS_DISTRO}/include/rosidl_typesupport_cpp"
    "/opt/ros/${ROS_DISTRO}/include/rosidl_typesupport_interface"
    "/opt/ros/${ROS_DISTRO}/include/rosidl_typesupport_introspection_c"
    "/opt/ros/${ROS_DISTRO}/include/rosidl_typesupport_introspection_cpp"
    "/opt/ros/${ROS_DISTRO}/include/statistics_msgs"
    "/opt/ros/${ROS_DISTRO}/include/tracetools"
    "/opt/ros/${ROS_DISTRO}/include/rcl_interfaces"
    "/opt/ros/${ROS_DISTRO}/include/sensor_msgs"
)

# 设置ROS2库路径
set(ROS2_LIBRARY_DIRS
    "/opt/ros/${ROS_DISTRO}/lib"
)

# 设置Dynamixel SDK路径
set(SDK_DIR "/usr/local")
set(SDK_INCLUDE_DIRS "${SDK_DIR}/include/dynamixel_sdk")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
  set(SDK_LIBRARIES "dxl_sbc_cpp")
  set(SDK_LIBRARY_DIRS "${SDK_DIR}/lib/linux_sbc")
else()
  set(SDK_LIBRARIES "dxl_x64_cpp")
  set(SDK_LIBRARY_DIRS "${SDK_DIR}/lib/linux64")
endif()

# 包含目录
include_directories(
  include
  ${SDK_INCLUDE_DIRS}
  ${ROS2_INCLUDE_DIRS}
)

# 链接目录
link_directories(
  ${SDK_LIBRARY_DIRS}
  ${ROS2_LIBRARY_DIRS}
)

# 添加可执行文件
add_executable(linkerta 
  src/linkerta.cpp
  src/main.cpp
)

# 链接库 - 使用正确的CMake变量
target_link_libraries(linkerta 
  ${SDK_LIBRARIES} 
  pthread
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}  # 新增：链接sensor_msgs库
)

# 安装目标
install(TARGETS linkerta
  DESTINATION lib/${PROJECT_NAME}
)

# ament包导出
ament_package()