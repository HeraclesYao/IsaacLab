### 安装依赖
```bash
1、安装 SDK
    cd linkerta_cpp/env/
    unzip sdk_cpp.zip
    cd sdk_cpp/build/linux64/
    make
    sudo make install
```
### 编译项目
```bash
    ./build.sh
```
### 配置
```bash
1、更新startup.sh中PASSWORD（电脑密码）。

2、标定遥操臂
    打开src/main.cpp文件，配置calibration: 0:关闭标定  1:打开标定
    1、打开标定。
    2、将遥操臂移动到要标定的姿态。
    3、单独启动程序
        ./build/linkerta
    4、将终端输出的14个关节数据复制并覆盖main.cpp中linkerta_offset的参数,并关闭标定。
    5、重新启动程序
        ./build/linkerta
    6、输出数据和被控设备关节相反：配置main.cpp中linkerta_negation参数中对应关节位置的值即可。

```
### 运行 
```bash
    ./startup.sh
```

### 运行ros2 topics
```bash

# 首先source ROS2环境
source /opt/ros/humble/setup.bash
# 清理构建环境
rm -rf build/ install/ log/

# 重新构建
colcon build

# 然后source你的工作空间
source install/setup.bash

# 运行程序
ros2 run linkerta linkerta calibrate  # 标定模式
# 或
ros2 run linkerta linkerta normal     # 正常模式

ros2 topic echo /cb_arm_control_cmd

ros2 topic echo /cb_arm_control_cmd std_msgs/msg/Float64MultiArray
```
