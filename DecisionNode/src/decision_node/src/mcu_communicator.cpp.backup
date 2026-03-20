#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <decision_node/mcu_comm.hpp>
#include <thread>
#include <cstdio>

// CRC8 查表 - 与 MCU 端完全相同（初始值 0xFF）
static constexpr uint8_t CRC8_TABLE[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

class MCUCommunicator
{
public:
    MCUCommunicator() : nh_("~"), serial_port_(""), serial_baudrate_(115200), 
                       frame_buffer_index_(0)
    {
        // 读取参数
        nh_.param("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
        nh_.param("baudrate", serial_baudrate_, 115200);
        
        // 读取导航发布频率 (默认50Hz)
        double nav_frequency = 50.0;
        nh_.param("nav_frequency", nav_frequency, 50.0);
        double nav_period = 1.0 / nav_frequency;  // 转换为周期(秒)
        
        pub_game_progress_ = nh_.advertise<std_msgs::UInt8>("/referee/game_progress", 1);
        pub_remain_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/remain_hp", 1);
        pub_bullet_remain_ = nh_.advertise<std_msgs::UInt16>("/referee/bullet_remain", 1);
        pub_occupy_status_ = nh_.advertise<std_msgs::UInt8>("/referee/occupy_status", 1);
        
        pub_robot_id_ = nh_.advertise<std_msgs::UInt8>("/robot/robot_id", 1);
        pub_robot_color_ = nh_.advertise<std_msgs::UInt8>("/robot/robot_color", 1);
        pub_self_hp_ = nh_.advertise<std_msgs::UInt16>("/robot/self_hp", 1);
        pub_self_max_hp_ = nh_.advertise<std_msgs::UInt16>("/robot/self_max_hp", 1);
        
        pub_red_1_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/red_1_hp", 1);
        pub_red_3_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/red_3_hp", 1);
        pub_red_7_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/red_7_hp", 1);
        pub_blue_1_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_1_hp", 1);
        pub_blue_3_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_3_hp", 1);
        pub_blue_7_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_7_hp", 1);
        
        pub_red_dead_ = nh_.advertise<std_msgs::UInt16>("/referee/red_dead", 1);
        pub_blue_dead_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_dead", 1);
        
        pub_friendly_score_ = nh_.advertise<std_msgs::Int32>("/referee/friendly_score", 1);
        pub_enemy_score_ = nh_.advertise<std_msgs::Int32>("/referee/enemy_score", 1);
        
        pub_enemy_hero_ = nh_.advertise<geometry_msgs::Point>("/enemy/hero_position", 1);
        pub_enemy_engineer_ = nh_.advertise<geometry_msgs::Point>("/enemy/engineer_position", 1);
        pub_enemy_standard_3_ = nh_.advertise<geometry_msgs::Point>("/enemy/standard_3_position", 1);
        pub_enemy_standard_4_ = nh_.advertise<geometry_msgs::Point>("/enemy/standard_4_position", 1);
        pub_enemy_sentry_ = nh_.advertise<geometry_msgs::Point>("/enemy/sentry_position", 1);
        pub_suggested_target_ = nh_.advertise<std_msgs::UInt8>("/radar/suggested_target", 1);
        // pub_radar_flags_ = nh_.advertise<std_msgs::UInt16>("/radar/radar_flags", 1);
        
        sub_motion_ = nh_.subscribe<std_msgs::UInt8>("/motion", 1, 
                                                     &MCUCommunicator::motionCallback, this);
        sub_recover_ = nh_.subscribe<std_msgs::UInt8>("/recover", 1,
                                                      &MCUCommunicator::recoverCallback, this);
        sub_bullet_up_ = nh_.subscribe<std_msgs::UInt8>("/bullet_up", 1,
                                                        &MCUCommunicator::bulletUpCallback, this);
        sub_bullet_num_ = nh_.subscribe<std_msgs::UInt8>("/bullet_num", 1,
                                                         &MCUCommunicator::bulletNumCallback, this);
        sub_navigation_ = nh_.subscribe<geometry_msgs::Vector3>("/navigation", 1,
                                                               &MCUCommunicator::navigationCallback, this);
        sub_nav_received_ = nh_.subscribe<std_msgs::UInt8>("/nav_received", 1,
                                                          &MCUCommunicator::navReceivedCallback, this);
        sub_dstar_status_ = nh_.subscribe<std_msgs::Bool>("/dstar_status", 1,
                                                         &MCUCommunicator::dstarStatusCallback, this);
        sub_cmd_vel_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,
                                                          &MCUCommunicator::cmdVelCallback, this);
        
        // 创建导航命令定时器
        navigation_timer_ = nh_.createTimer(ros::Duration(nav_period),
                                            &MCUCommunicator::navigationTimerCallback, this);
        
        try
        {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(serial_baudrate_);
            serial_.setBytesize(serial::eightbits);      // 数据位：8
            serial_.setParity(serial::parity_none);      // 校验位：None
            serial_.setStopbits(serial::stopbits_one);   // 停止位：1
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
            
            if (serial_.isOpen())
            {
                ROS_INFO("MCU Serial port opened successfully: %s @ %d baud", 
                         serial_port_.c_str(), serial_baudrate_);
            }
            else
            {
                ROS_ERROR("Failed to open serial port: %s", serial_port_.c_str());
                ROS_ERROR("Debugging steps:");
                ROS_ERROR("   1. Check device exists: ls -la %s", serial_port_.c_str());
                ROS_ERROR("   2. Check permissions: stat %s", serial_port_.c_str());
            }
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Serial exception during port setup: %s", e.what());
        }
        
        recv_thread_ = std::thread(&MCUCommunicator::receiveThread, this);
    }
    
    ~MCUCommunicator()
    {
        if (recv_thread_.joinable())
        {
            recv_thread_.join();
        }
        if (serial_.isOpen())
        {
            serial_.close();
        }
    }
    
private:
    ros::NodeHandle nh_;
    serial::Serial serial_;
    std::string serial_port_;
    int serial_baudrate_;
    
    // ROS 发布者
    ros::Publisher pub_game_progress_;
    ros::Publisher pub_remain_hp_;
    ros::Publisher pub_bullet_remain_;
    ros::Publisher pub_occupy_status_;
    
    ros::Publisher pub_robot_id_;
    ros::Publisher pub_robot_color_;
    ros::Publisher pub_self_hp_;
    ros::Publisher pub_self_max_hp_;
    
    ros::Publisher pub_red_1_hp_;
    ros::Publisher pub_red_3_hp_;
    ros::Publisher pub_red_7_hp_;
    ros::Publisher pub_blue_1_hp_;
    ros::Publisher pub_blue_3_hp_;
    ros::Publisher pub_blue_7_hp_;
    
    ros::Publisher pub_red_dead_;
    ros::Publisher pub_blue_dead_;
    
    ros::Publisher pub_friendly_score_;
    ros::Publisher pub_enemy_score_;
    
    ros::Publisher pub_enemy_hero_;
    ros::Publisher pub_enemy_engineer_;
    ros::Publisher pub_enemy_standard_3_;
    ros::Publisher pub_enemy_standard_4_;
    ros::Publisher pub_enemy_sentry_;
    ros::Publisher pub_suggested_target_;
    ros::Publisher pub_radar_flags_;
    
    // ROS 订阅者
    ros::Subscriber sub_motion_;
    ros::Subscriber sub_recover_;
    ros::Subscriber sub_bullet_up_;
    ros::Subscriber sub_bullet_num_;
    ros::Subscriber sub_navigation_;
    ros::Subscriber sub_nav_received_;
    ros::Subscriber sub_dstar_status_;
    ros::Subscriber sub_cmd_vel_;
    ros::Timer navigation_timer_;
    
    // 发送缓冲
    uint8_t tx_buffer_[256];
    size_t tx_buffer_index_;
    
    // 接收缓冲
    uint8_t frame_buffer_[MCU_FRAME_SIZE];
    size_t frame_buffer_index_;
    std::thread recv_thread_;
    
    uint8_t current_hp_up_ = 0;
    uint8_t current_bullet_up_ = 0;
    uint8_t current_bullet_num_ = 0;
    uint8_t current_motion_mode_ = 0;
    
    // 导航数据变量
    float current_nav_vx_ = 0.0f;
    float current_nav_vy_ = 0.0f;
    float current_nav_z_angle_ = 0.0f;
    uint8_t current_nav_received_ = 0;
    uint8_t current_nav_arrived_ = 0;
    
    // 敌方位置缓存 - 用于处理-8888无效值
    float cached_enemy_hero_x_ = 0.0f;
    float cached_enemy_hero_y_ = 0.0f;
    float cached_enemy_engineer_x_ = 0.0f;
    float cached_enemy_engineer_y_ = 0.0f;
    float cached_enemy_standard_3_x_ = 0.0f;
    float cached_enemy_standard_3_y_ = 0.0f;
    float cached_enemy_standard_4_x_ = 0.0f;
    float cached_enemy_standard_4_y_ = 0.0f;
    float cached_enemy_sentry_x_ = 0.0f;
    float cached_enemy_sentry_y_ = 0.0f;
    
    // 分数追踪变量
    int32_t friendly_score_ = 200;  // 己方初始分数
    int32_t enemy_score_ = 200;     // 敌方初始分数
    ros::Time last_score_update_time_;  // 上次更新分数的时间
    int last_occupy_status_ = 0;    // 上一帧的占领状态
    uint16_t last_red_dead_ = 0;    // 上一帧红方死亡状态
    uint16_t last_blue_dead_ = 0;   // 上一帧蓝方死亡状态
    uint8_t robot_color_ = 0;       // 0=red, 1=blue
    
    // Motion回调函数 
    void motionCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        // ROS_INFO("motionCallback triggered: motion_mode=%u", msg->data);
        sendMotionCommand(msg->data);
    }

    // Recover
    void recoverCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_hp_up_ = (msg->data != 0) ? 1 : 0;
        sendMotionCommand(current_motion_mode_);
    }
    
    // Bullet
    void bulletUpCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_bullet_up_ = (msg->data != 0) ? 1 : 0;
        sendMotionCommand(current_motion_mode_);
    }
    
    // Bullet Num
    void bulletNumCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_bullet_num_ = msg->data;
        sendMotionCommand(current_motion_mode_);
    }
    
    // Navigation
    void navigationCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        sendNavigationCommand(msg->x, msg->y, msg->z);
    }
    
    // Nav Received
    void navReceivedCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_nav_received_ = msg->data;
        // ROS_DEBUG("Nav received updated: received=%u", current_nav_received_);
    }
    
    // D* Status
    void dstarStatusCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        current_nav_arrived_ = msg->data ? 1 : 0;
        // ROS_DEBUG("D* status updated: arrived=%u", current_nav_arrived_);
    }
    
    // Cmd Vel: 订阅速度命令，更新导航数据变量
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // 提取线速度和角速度，只更新变量，由定时器固定频率发送
        current_nav_vx_ = msg->linear.x;
        current_nav_vy_ = msg->linear.y;
        current_nav_z_angle_ = msg->angular.z;
        
        ROS_DEBUG("CmdVel received: vx=%.4f, vy=%.4f, z_angle=%.4f", 
                  current_nav_vx_, current_nav_vy_, current_nav_z_angle_);
    }
    
    // 导航命令定时器回调 - 固定频率发送NavigationFrame到下位机
    void navigationTimerCallback(const ros::TimerEvent& event)
    {
        sendNavigationCommand(current_nav_vx_, current_nav_vy_, current_nav_z_angle_);
    }
    
    // 验证敌方坐标有效性（-8888为无效值）
    float validateEnemyCoordinate(float new_value, float cached_value)
    {
        // 如果新值为-8888（无效值），返回缓存的旧值
        if (new_value == -8888.0f)
        {
            return cached_value;
        }
        // 否则返回新值并更新缓存
        return new_value;
    }
    
    // 更新敌方位置数据，处理-8888无效值
    void updateEnemyPositions(const MCUDataFrame& frame)
    {
        // 验证并更新英雄位置
        cached_enemy_hero_x_ = validateEnemyCoordinate(frame.enemy_hero_x, cached_enemy_hero_x_);
        cached_enemy_hero_y_ = validateEnemyCoordinate(frame.enemy_hero_y, cached_enemy_hero_y_);
        
        // 验证并更新工程位置
        cached_enemy_engineer_x_ = validateEnemyCoordinate(frame.enemy_engineer_x, cached_enemy_engineer_x_);
        cached_enemy_engineer_y_ = validateEnemyCoordinate(frame.enemy_engineer_y, cached_enemy_engineer_y_);
        
        // 验证并更新步兵3位置
        cached_enemy_standard_3_x_ = validateEnemyCoordinate(frame.enemy_standard_3_x, cached_enemy_standard_3_x_);
        cached_enemy_standard_3_y_ = validateEnemyCoordinate(frame.enemy_standard_3_y, cached_enemy_standard_3_y_);
        
        // 验证并更新步兵4位置
        cached_enemy_standard_4_x_ = validateEnemyCoordinate(frame.enemy_standard_4_x, cached_enemy_standard_4_x_);
        cached_enemy_standard_4_y_ = validateEnemyCoordinate(frame.enemy_standard_4_y, cached_enemy_standard_4_y_);
        
        // 验证并更新哨兵位置
        cached_enemy_sentry_x_ = validateEnemyCoordinate(frame.enemy_sentry_x, cached_enemy_sentry_x_);
        cached_enemy_sentry_y_ = validateEnemyCoordinate(frame.enemy_sentry_y, cached_enemy_sentry_y_);
    }
    
    // 发送命令到下位机
    void sendMotionCommand(uint8_t motion_mode)
    {
        current_motion_mode_ = motion_mode;
        
        MotionCommandFrame frame;
        frame.sof = 0x92;              // 0x92
        frame.motion_mode_up = motion_mode;
        frame.hp_up = current_hp_up_;
        frame.bullet_up = current_bullet_up_;
        frame.bullet_num = current_bullet_num_;
        frame.eof = 0xFE;             // 0xFE
        
        // CRC8校验 (与 MCU 一致：计算 sof 到 bullet_num 的 CRC)
        // MotionCommandFrame 大小为 7 字节，CRC 计算前 5 字节（从 sof 开始，不包括 crc8 和 eof）
        frame.crc8 = calculateCRC8((uint8_t*)&frame.sof, sizeof(MotionCommandFrame) - 2, 0xFF);
        
        try
        {
            if (!serial_.isOpen())
            {
                ROS_ERROR("Serial port is CLOSED! Port: %s. Cannot send motion command.", 
                         serial_port_.c_str());
                return;
            }
            
            int written = serial_.write((uint8_t*)&frame, sizeof(frame));
            
            // 验证写入是否成功
            if (written == (int)sizeof(frame))
            {
                // ROS_INFO("Motion command sent: motion_mode=%u, hp_up=%u, bullet_up=%u, bullet_num=%u", 
                //          motion_mode, current_hp_up_, current_bullet_up_, current_bullet_num_);
            }
            else if (written > 0)
            {
                ROS_WARN("Partial write: expected %zu bytes, but only wrote %d bytes", sizeof(frame), written);
            }
            else
            {
                ROS_ERROR("Write failed: write returned %d", written);
            }
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Serial exception during write: %s", e.what());
        }
    }
    
    // 发送导航命令到下位机
    void sendNavigationCommand(float vx, float vy, float z_angle)
    {
        current_nav_vx_ = vx;
        current_nav_vy_ = vy;
        current_nav_z_angle_ = z_angle;
        
        NavigationFrame frame;
        frame.sof = 0x93;              // 0x93
        frame.vx = vx;
        frame.vy = vy;
        frame.z_angle = z_angle;
        frame.received = current_nav_received_;  
        frame.arrived = current_nav_arrived_;   
        frame.eof = 0xFE;             // 0xFE
        
        // CRC8校验 (计算 sof 到 arrived 的 CRC)
        // NavigationFrame 大小为 17 字节，CRC 计算前 15 字节（从 sof 开始，不包括 crc8 和 eof）
        frame.crc8 = calculateCRC8((uint8_t*)&frame.sof, sizeof(NavigationFrame) - 2, 0xFF);
        
        try
        {
            if (!serial_.isOpen())
            {
                ROS_ERROR("Serial port is CLOSED! Cannot send navigation command.");
                return;
            }
            
            int written = serial_.write((uint8_t*)&frame, sizeof(frame));
            
            if (written == (int)sizeof(frame))
            {
                // ROS_INFO("Navigation command sent: vx=%.4f, vy=%.4f, z_angle=%.4f", vx, vy, z_angle);
            }
            else if (written > 0)
            {
                ROS_WARN("Partial write: expected %zu bytes, but only wrote %d bytes", sizeof(frame), written);
            }
            else
            {
                ROS_ERROR("Write failed: write returned %d", written);
            }
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Serial exception during navigation write: %s", e.what());
        }
    }
    
    // CRC8 查表实现
    uint8_t calculateCRC8(const uint8_t* pch_message, size_t dw_length, uint8_t ucCRC8 = 0xFF)
    {
        unsigned char uc_index;
        while (dw_length--)
        {
            uc_index = ucCRC8 ^ (*pch_message++);
            ucCRC8 = CRC8_TABLE[uc_index];
        }
        return ucCRC8;
    }
    
    // CRC8验证函数
    bool verifyCRC8(MCUDataFrame* frame)
    {
        uint8_t received_crc = frame->crc8;
        
        // 按照 MCU 的方式计算：get_CRC8_check_sum((uint8_t*)&frame, MCU_FRAME_SIZE - 2, 0xFF)
        // MCU_FRAME_SIZE = 46, 所以计算前 44 字节的 CRC
        uint8_t calculated_crc = calculateCRC8((uint8_t*)&frame->sof, MCU_FRAME_SIZE - 2, 0xFF);
        
        if (received_crc != calculated_crc)
        {
            ROS_WARN("CRC8 mismatch: received=0x%02X, calculated=0x%02X", received_crc, calculated_crc);
            return false;
        }
        return true;
    }
    
    void receiveThread()
    {
        ros::Rate loop_rate(100);  // 100Hz
        
        while (ros::ok())
        {
            if (!serial_.isOpen())
            {
                // 尝试重新连接
                try
                {
                    serial_.open();
                    ROS_INFO("Reconnected to serial port");
                }
                catch (const serial::SerialException& e)
                {
                    ROS_WARN("Failed to reconnect: %s", e.what());
                    loop_rate.sleep();
                    continue;
                }
            }
            
            try
            {
                size_t available = serial_.available();
                if (available > 0)
                {
                    std::string data = serial_.read(available);
                    std::vector<uint8_t> buffer(data.begin(), data.end());
                    processReceivedData(buffer);
                }
            }
            catch (const serial::SerialException& e)
            {
                ROS_ERROR("Serial read exception: %s", e.what());
                if (serial_.isOpen())
                {
                    serial_.close();
                }
            }
            
            loop_rate.sleep();
        }
    }
    
    void processReceivedData(const std::vector<uint8_t>& data)
    {
        for (uint8_t byte : data)
        {
            // 寻找帧头
            if (frame_buffer_index_ == 0)
            {
                if (byte == MCU_FRAME_SOF)
                {
                    frame_buffer_[frame_buffer_index_++] = byte;
                }
                continue;
            }
            
            // 接收数据
            frame_buffer_[frame_buffer_index_++] = byte;
            
            // 检查是否接收完整帧
            if (frame_buffer_index_ == MCU_FRAME_SIZE)
            {
                // 验证帧尾
                if (frame_buffer_[MCU_FRAME_SIZE - 1] == MCU_FRAME_EOF)
                {
                    // 解析并发布数据
                    parseAndPublish();
                }
                else
                {
                    // ROS_DEBUG("Invalid frame end marker: 0x%02X (expected 0xFE at position %u).", 
                    //          frame_buffer_[MCU_FRAME_SIZE - 1], MCU_FRAME_SIZE - 1);
                    
                    // 尝试重新同步：寻找缓冲区中的下一个帧头
                    bool found_resync = false;
                    for (size_t i = 1; i < MCU_FRAME_SIZE; i++)
                    {
                        if (frame_buffer_[i] == MCU_FRAME_SOF)
                        {
                            // ROS_DEBUG("Found potential frame resync at offset %zu", i);
                            // 将缓冲区数据移动以对齐新的帧头
                            memmove(frame_buffer_, frame_buffer_ + i, MCU_FRAME_SIZE - i);
                            frame_buffer_index_ = MCU_FRAME_SIZE - i;
                            found_resync = true;
                            break;
                        }
                    }
                    
                    if (!found_resync)
                    {
                        frame_buffer_index_ = 0;  // 无法重新同步，重置缓冲区
                    }
                }
                
                // 如果成功解析，重置缓冲区
                if (frame_buffer_[MCU_FRAME_SIZE - 1] == MCU_FRAME_EOF)
                {
                    frame_buffer_index_ = 0;
                }
            }
        }
    }
    
    void parseAndPublish()
    {
        // 将原始字节数据复制到结构体
        MCUDataFrame frame;
        memcpy(&frame, frame_buffer_, MCU_FRAME_SIZE);
        
        // 验证帧头和帧尾
        if (frame.sof != MCU_FRAME_SOF || frame.eof != MCU_FRAME_EOF)
        {
            ROS_WARN("Invalid frame markers: SOF=0x%02X (expected 0x%02X), EOF=0x%02X (expected 0x%02X)", 
                    frame.sof, MCU_FRAME_SOF, frame.eof, MCU_FRAME_EOF);
            return;
        }
        
        // 验证CRC8校验
        if (!verifyCRC8(&frame))
        {
            ROS_WARN("CRC8 verification failed - Frame details: "
                    "robot_id=%u, game_progress=%u, crc8=0x%02X, EOF=0x%02X",
                    frame.robot_id, frame.game_progress, frame.crc8, frame.eof);
            return;
        }
        
        // ROS_DEBUG("Valid frame received: robot_id=%u, game_progress=%u, crc8=0x%02X",
        //          frame.robot_id, frame.game_progress, frame.crc8);
        
        // 更新敌方位置数据（处理-8888无效值）
        updateEnemyPositions(frame);
        
        // 更新机器人颜色（0=red, 1=blue）
        robot_color_ = frame.robot_color;
        
        // 发布已有topic的数据
        std_msgs::UInt8 msg_uint8;
        std_msgs::UInt16 msg_uint16;
        std_msgs::Float32 msg_float;
   
        msg_uint8.data = frame.game_progress;
        pub_game_progress_.publish(msg_uint8);
        
        msg_uint16.data = frame.self_hp;
        pub_remain_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.bullet_remain;
        pub_bullet_remain_.publish(msg_uint16);
        
        msg_uint8.data = frame.occupy_status;
        pub_occupy_status_.publish(msg_uint8);
        
        
        msg_uint8.data = frame.robot_id;
        pub_robot_id_.publish(msg_uint8);
        
        msg_uint8.data = frame.robot_color;
        pub_robot_color_.publish(msg_uint8);
        
        msg_uint16.data = frame.self_hp;
        pub_self_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.self_max_hp;
        pub_self_max_hp_.publish(msg_uint16);
        
        
        msg_uint16.data = frame.red_1_hp;
        pub_red_1_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.red_3_hp;
        pub_red_3_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.red_7_hp;
        pub_red_7_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.blue_1_hp;
        pub_blue_1_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.blue_3_hp;
        pub_blue_3_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.blue_7_hp;
        pub_blue_7_hp_.publish(msg_uint16);
        
        // 发布敌方位置数据
        geometry_msgs::Point enemy_pos;
        
        enemy_pos.x = cached_enemy_hero_x_;
        enemy_pos.y = cached_enemy_hero_y_;
        enemy_pos.z = 0.0f;
        pub_enemy_hero_.publish(enemy_pos);
        
        enemy_pos.x = cached_enemy_engineer_x_;
        enemy_pos.y = cached_enemy_engineer_y_;
        enemy_pos.z = 0.0f;
        pub_enemy_engineer_.publish(enemy_pos);
        
        enemy_pos.x = cached_enemy_standard_3_x_;
        enemy_pos.y = cached_enemy_standard_3_y_;
        enemy_pos.z = 0.0f;
        pub_enemy_standard_3_.publish(enemy_pos);
        
        enemy_pos.x = cached_enemy_standard_4_x_;
        enemy_pos.y = cached_enemy_standard_4_y_;
        enemy_pos.z = 0.0f;
        pub_enemy_standard_4_.publish(enemy_pos);
        
        enemy_pos.x = cached_enemy_sentry_x_;
        enemy_pos.y = cached_enemy_sentry_y_;
        enemy_pos.z = 0.0f;
        pub_enemy_sentry_.publish(enemy_pos);
        
        msg_uint8.data = frame.suggested_target;
        pub_suggested_target_.publish(msg_uint8);
        
        msg_uint16.data = frame.radar_flags;
        pub_radar_flags_.publish(msg_uint16);
        
        msg_uint16.data = frame.red_dead;
        pub_red_dead_.publish(msg_uint16);
        
        msg_uint16.data = frame.blue_dead;
        pub_blue_dead_.publish(msg_uint16);
        
        updateScore(frame);
        
        
        std_msgs::Int32 msg_score;
        msg_score.data = friendly_score_;
        pub_friendly_score_.publish(msg_score);
        msg_score.data = enemy_score_;
        pub_enemy_score_.publish(msg_score);
        
        ROS_DEBUG("MCU frame parsed: game_progress=%u, self_hp=%u, bullet=%u, friendly_score=%d, enemy_score=%d",
                 frame.game_progress, frame.self_hp, frame.bullet_remain, friendly_score_, enemy_score_);
    }
    //分数计算部分
    void updateScore(const MCUDataFrame& frame)
    {
        // 只有在游戏开始(game_progress == 4)时才计算分数
        // 否则分数保持在200不变
        if (frame.game_progress != 4)
        {
            friendly_score_ = 200;
            enemy_score_ = 200;
            // 重置时间戳，为下一次游戏开始做准备
            last_score_update_time_ = ros::Time(0);
            last_occupy_status_ = 0;
            last_red_dead_ = 0;
            last_blue_dead_ = 0;
            return;
        }
        
        ros::Time current_time = ros::Time::now();
        
        // 初始化时间戳
        if (last_score_update_time_.isZero())
        {
            last_score_update_time_ = current_time;
        }
        
        // 检测占领状态变化 - 每秒扣1分
        if (frame.occupy_status != last_occupy_status_)
        {
            // occupy_statu 2 到 3 的情况 - 己方扣分
            if (last_occupy_status_ == 2 && frame.occupy_status == 3)
            {
                friendly_score_ = std::max(0, friendly_score_ - 1);
                ROS_INFO("Occupy status changed from 2 to 3: friendly_score now %d", friendly_score_);
            }
            //  occupy_status  1 到 3  - 敌方扣分
            else if (last_occupy_status_ == 1 && frame.occupy_status == 3)
            {
                enemy_score_ = std::max(0, enemy_score_ - 1);
                ROS_INFO("Occupy status changed from 1 to 3: enemy_score now %d", enemy_score_);
            }
            
            last_occupy_status_ = frame.occupy_status;
            last_score_update_time_ = current_time;
        }
        else if ((current_time - last_score_update_time_).toSec() >= 1.0)
        {
            // occupy_status == 2: 对方占领 → 己方扣1分
            if (frame.occupy_status == 2)
            {
                friendly_score_ = std::max(0, friendly_score_ - 1);
            }
            // occupy_status == 1: 己方占领 → 对方扣1分
            else if (frame.occupy_status == 1)
            {
                enemy_score_ = std::max(0, enemy_score_ - 1);
            }
            last_score_update_time_ = current_time;
        }
        
        // 检测红方死亡状态变化
        if (frame.red_dead != last_red_dead_)
        {
            uint16_t new_deaths = frame.red_dead - last_red_dead_;
            for (uint16_t i = 0; i < new_deaths; i++)
            {
                if (robot_color_ == 0)  // 己方是红色
                {
                    // 己方被击杀，自己人扣20分
                    friendly_score_ = std::max(0, friendly_score_ - 20);
                    ROS_INFO("Red robot killed (friendly): friendly_score now %d", friendly_score_);
                }
                else  // 己方是蓝色
                {
                    // 击杀对方，敌方扣20分
                    enemy_score_ = std::max(0, enemy_score_ - 20);
                    ROS_INFO("Red robot killed (enemy): enemy_score now %d", enemy_score_);
                }
            }
            last_red_dead_ = frame.red_dead;
        }
        
        // 检测蓝方死亡状态变化
        if (frame.blue_dead != last_blue_dead_)
        {
            uint16_t new_deaths = frame.blue_dead - last_blue_dead_;
            for (uint16_t i = 0; i < new_deaths; i++)
            {
                if (robot_color_ == 1)  // 己方是蓝色
                {
                    // 己方被击杀，自己人扣20分
                    friendly_score_ = std::max(0, friendly_score_ - 20);
                    ROS_INFO("Blue robot killed (friendly): friendly_score now %d", friendly_score_);
                }
                else  // 己方是红色
                {
                    // 击杀对方，敌方扣20分
                    enemy_score_ = std::max(0, enemy_score_ - 20);
                    ROS_INFO("Blue robot killed (enemy): enemy_score now %d", enemy_score_);
                }
            }
            last_blue_dead_ = frame.blue_dead;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mcu_communicator");
    
    try
    {
        MCUCommunicator comm;
        ROS_INFO("MCU Communicator node started");
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Fatal error: %s", e.what());
        return 1;
    }
    
    return 0;
}
