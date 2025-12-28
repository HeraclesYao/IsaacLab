#include <iostream>
#include <cstring>
#include <thread>
#include <cmath>
#include <csignal>
#include <chrono>
#include <vector>
#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <fstream>
#include <sstream>
#include "linkerta.hpp"

// ROS2头文件 - 修改为使用JointState类型
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>  // 修改：使用JointState类型

std::atomic<bool> running{true};

class LinkerTA : public rclcpp::Node
{
public:
    LinkerTA(int calibration_mode) : Node("linkerta_node")
    {
        // 创建发布者 - 修改为使用JointState类型和正确的主题名称
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/cb_arm_control_cmd", 10);  // 修改：使用JointState类型和标准主题名称
        
        calibration = calibration_mode;
        
        // 初始化标定数组为适当大小
        linkerta_offset.resize(14, 0.0);
        linkerta_negation.resize(14, 1.0);
        
        // 尝试加载已有的标定文件
        if (!loadCalibrationFromFile()) {
            // 如果没有标定文件，使用默认值
            if (calibration == 0) {
                // 正常模式：使用默认标定值
                linkerta_offset = {175.957, 271.582, 127.002, 86.8359, 135.879, 180, 177.715, 183.076, 84.375, 223.066, 179.385, 226.582, 172.705, 86.2207};
                linkerta_negation = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
                RCLCPP_WARN(this->get_logger(), "未找到标定文件，使用默认标定值");
            } else {
                // 标定模式：零偏移
                linkerta_offset = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
                linkerta_negation = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
            }
        }
        
        if (calibration == 0) {
            RCLCPP_INFO(this->get_logger(), "运行模式：正常模式（使用标定值）");
        } else {
            RCLCPP_INFO(this->get_logger(), "运行模式：标定模式（获取原始值）");
        }
    }
    
    ~LinkerTA()
    {
        running = false;
    }
    
    void process()
    {
        const int targetFrequency = 500;
        const auto interval = std::chrono::duration<double, std::milli>(1000.0 / targetFrequency);
        
        uint8_t num_motors = 14;
        uint8_t motor_ids[] = {1, 2, 3, 4, 5, 6, 7};
        std::string port_name = "/dev/ttyUSB0";
        int baudrate = 3000000;
        MasterArm master_arm(num_motors, motor_ids, port_name, baudrate);
        
        // 标定模式：记录初始位置并保存
        bool calibration_saved = false;
        if (calibration == 1) {
            std::vector<double> initial_positions;
            
            RCLCPP_INFO(this->get_logger(), "开始标定，请将机械臂置于零位...");
            std::this_thread::sleep_for(std::chrono::seconds(5)); // 增加等待时间
            
            // 获取初始位置
            initial_positions = master_arm.getJointsPosition();
            if (initial_positions.empty()) {
                RCLCPP_ERROR(this->get_logger(), "无法获取关节位置，请检查机械臂连接");
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "获取到初始位置，正在保存标定文件...");
            
            // 保存标定值
            if (saveCalibrationToFile(initial_positions)) {
                RCLCPP_INFO(this->get_logger(), "标定文件保存成功！");
                calibration_saved = true;
                
                // 更新当前标定值
                linkerta_offset = initial_positions;
                linkerta_negation = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
            } else {
                RCLCPP_ERROR(this->get_logger(), "标定文件保存失败！");
            }
        }
        
        while (rclcpp::ok() && running) {
            std::vector<double> raw_position = master_arm.getJointsPosition();
            if (raw_position.empty()) {
                RCLCPP_ERROR(this->get_logger(), "无法获取关节位置");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            
            // 应用标定（如果处于正常模式）
            std::vector<double> calibrated_position = applyCalibration(raw_position);
            
            // 发布标定后的数据到ROS2 topic
            publishArmData(calibrated_position);
            
            // 输出调试信息
            if (calibration == 1) {
                if (!calibration_saved) {
                    // 如果标定未保存，显示原始值
                    std::cout << "=== 标定模式 ===" << std::endl;
                    std::cout << "原始关节位置（用于标定）：" << std::endl;
                    for (int i = 0; i < raw_position.size(); i++) {
                        std::cout << "关节 " << i << " : " << raw_position[i] << " 度" << std::endl;
                    }
                    std::cout << "----------------------" << std::endl;
                } else {
                    // 标定已保存，显示标定后的值
                    std::cout << "=== 标定完成 ===" << std::endl;
                    std::cout << "标定后的关节角度：" << std::endl;
                    for (int i = 0; i < calibrated_position.size(); i++) {
                        std::cout << "关节 " << i << " : " << calibrated_position[i] << " 度" << std::endl;
                    }
                    std::cout << "----------------------" << std::endl;
                }
            } else {
                std::cout << "=== 正常模式 ===" << std::endl;
                std::cout << "标定后的关节角度：" << std::endl;
                for (int i = 0; i < calibrated_position.size(); i++) {
                    std::cout << "关节 " << i << " : " << calibrated_position[i] << " 度" << std::endl;
                }
                std::cout << "----------------------" << std::endl;
            }
            
            std::this_thread::sleep_for(interval);
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;  // 修改：使用JointState类型
    int calibration;
    std::vector<double> linkerta_offset;    // 添加成员变量声明
    std::vector<double> linkerta_negation;  // 添加成员变量声明
    
    // 保存标定值到文件
    bool saveCalibrationToFile(const std::vector<double>& offsets)
    {
        std::ofstream file("calibration.txt");
        if (!file.is_open()) {
            return false;
        }
        
        file << "# LinkerTA 标定文件" << std::endl;
        file << "# 生成时间: " << getCurrentTime() << std::endl;
        file << "# 关节数量: " << offsets.size() << std::endl;
        file << "# 格式: 关节索引,偏移值,方向系数" << std::endl;
        
        for (size_t i = 0; i < offsets.size(); i++) {
            file << i << "," << offsets[i] << ",1" << std::endl;
        }
        
        file.close();
        return true;
    }
    
    // 从文件加载标定值
    bool loadCalibrationFromFile()
    {
        std::ifstream file("calibration.txt");
        if (!file.is_open()) {
            return false;
        }
        
        std::string line;
        linkerta_offset.clear();
        linkerta_negation.clear();
        
        while (std::getline(file, line)) {
            // 跳过注释行
            if (line.empty() || line[0] == '#') {
                continue;
            }
            
            std::istringstream iss(line);
            std::string token;
            std::vector<std::string> tokens;
            
            while (std::getline(iss, token, ',')) {
                tokens.push_back(token);
            }
            
            if (tokens.size() >= 2) {
                try {
                    linkerta_offset.push_back(std::stod(tokens[1]));
                    if (tokens.size() >= 3) {
                        linkerta_negation.push_back(std::stod(tokens[2]));
                    } else {
                        linkerta_negation.push_back(1.0);
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "解析标定文件错误: %s", e.what());
                    return false;
                }
            }
        }
        
        file.close();
        return linkerta_offset.size() > 0;
    }
    
    // 获取当前时间字符串
    std::string getCurrentTime()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::string time_str = std::ctime(&time_t);
        time_str.pop_back(); // 移除换行符
        return time_str;
    }
    
    // 应用标定函数 - 正确的标定逻辑
    std::vector<double> applyCalibration(const std::vector<double>& raw_position)
    {
        std::vector<double> calibrated;
        for (size_t i = 0; i < raw_position.size(); i++) {
            if (i < linkerta_offset.size()) {
                // 正确的标定逻辑：实际角度 - 标定偏移值
                double calibrated_value = (raw_position[i] - linkerta_offset[i]) * linkerta_negation[i];
                calibrated.push_back(calibrated_value);
            } else {
                calibrated.push_back(raw_position[i]);
            }
        }
        return calibrated;
    }
    
    void publishArmData(const std::vector<double>& position)
    {
        auto message = sensor_msgs::msg::JointState();  // 修改：使用JointState类型
        
        // 设置时间戳
        message.header.stamp = this->now();
        message.header.frame_id = "linkerta_frame";
        
        // 设置关节名称（14个关节，与IsaacLab配置一致）
        message.name = {
            "joint_0",//"left_shoulder_pitch_joint",     // 左肩俯仰
            "joint_1",//"left_shoulder_roll_joint",      // 左肩滚动
            "joint_2",//"left_shoulder_yaw_joint",       // 左肩偏航
            "joint_3",//"left_elbow_joint",              // 左肘
            "joint_4",//"left_wrist_yaw_joint",          // 左肘偏航
            "joint_5",//"left_wrist_roll_joint",         // 左腕滚动
            "joint_6",//"left_wrist_pitch_joint"         // 左腕俯仰

            "joint_7",//"right_shoulder_pitch_joint",    // 右肩俯仰
            "joint_8",//"right_shoulder_roll_joint",     // 右肩滚动
            "joint_9",//"right_shoulder_yaw_joint",      // 右肩偏航
            "joint_10",//"right_elbow_joint",            // 右肘
            "joint_11",//"right_wrist_yaw_joint",        // 右腕偏航
            "joint_12",//"right_wrist_roll_joint",       // 右腕滚动
            "joint_13",//"right_wrist_pitch_joint",      // 右腕俯仰
        };
        
        // 填充关节位置数据（弧度）
        for (const auto& pos : position) {
            message.position.push_back(pos * M_PI / 180.0);  // 将角度转换为弧度
        }
        
        // 设置速度和力（可选，设为0）
        message.velocity.resize(position.size(), 0.0);
        message.effort.resize(position.size(), 0.0);
        
        // 发布消息
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published %zu joint positions to /cb_arm_control_cmd", position.size());
    }
};

void signalHandler(int signal)
{
    if (signal == SIGINT) {
        running = false;
    }
}

void printUsage()
{
    std::cout << "用法：" << std::endl;
    std::cout << "  ./linkerta [模式]" << std::endl;
    std::cout << "模式：" << std::endl;
    std::cout << "  0 或 normal  - 正常模式（使用标定值）" << std::endl;
    std::cout << "  1 或 calibrate - 标定模式（获取并保存标定值）" << std::endl;
    std::cout << "标定流程：" << std::endl;
    std::cout << "  1. 运行标定模式: ros2 run linkerta linkerta calibrate" << std::endl;
    std::cout << "  2. 将机械臂置于零位，程序会自动保存标定值到 calibration.txt" << std::endl;
    std::cout << "  3. 运行正常模式: ros2 run linkerta linkerta normal" << std::endl;
    std::cout << "  4. 程序会从 calibration.txt 加载标定值" << std::endl;
}

int main(int argc, char **argv)
{
    int calibration_mode = 0; // 默认正常模式
    
    // 解析命令行参数
    if (argc > 1) {
        std::string mode(argv[1]);
        if (mode == "1" || mode == "calibrate") {
            calibration_mode = 1;
        } else if (mode == "0" || mode == "normal") {
            calibration_mode = 0;
        } else {
            std::cerr << "错误：未知模式 '" << mode << "'" << std::endl;
            printUsage();
            return 1;
        }
    }
    
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    std::signal(SIGINT, signalHandler);
    auto node = std::make_shared<LinkerTA>(calibration_mode);
    
    // 在单独的线程中运行处理逻辑
    std::thread process_thread([node]() {
        node->process();
    });
    
    // 在主线程中运行ROS2 spin
    rclcpp::spin(node);
    
    // 清理
    running = false;
    if (process_thread.joinable()) {
        process_thread.join();
    }
    rclcpp::shutdown();
    
    return 0;
}