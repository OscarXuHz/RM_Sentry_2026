#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <serial/serial.h>
#include <decision_node/mcu_comm.hpp>
#include <thread>
#include <cstdio>
#include <cmath>
#include <algorithm>

class MCUCommunicator
{
public:
    MCUCommunicator() : nh_("~"), serial_port_(""), serial_baudrate_(115200),
                       frame_buffer_index_(0)
    {
        nh_.param("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
        nh_.param("baudrate", serial_baudrate_, 115200);
        
        double nav_frequency = 50.0;
        nh_.param("nav_frequency", nav_frequency, 50.0);
        double nav_period = 1.0 / nav_frequency;
        
        // ===== Publishers: game data from MCU =====
        pub_yaw_angle_ = nh_.advertise<std_msgs::Float32>("/mcu/yaw_angle", 1);
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
        pub_enemy_hero_ = nh_.advertise<geometry_msgs::Point>("/enemy/hero_position", 1);
        pub_enemy_engineer_ = nh_.advertise<geometry_msgs::Point>("/enemy/engineer_position", 1);
        pub_enemy_standard_3_ = nh_.advertise<geometry_msgs::Point>("/enemy/standard_3_position", 1);
        pub_enemy_standard_4_ = nh_.advertise<geometry_msgs::Point>("/enemy/standard_4_position", 1);
        pub_enemy_sentry_ = nh_.advertise<geometry_msgs::Point>("/enemy/sentry_position", 1);
        pub_suggested_target_ = nh_.advertise<std_msgs::UInt8>("/radar/suggested_target", 1);
        pub_radar_flags_ = nh_.advertise<std_msgs::UInt16>("/radar/radar_flags", 1);
        pub_can_free_revive_ = nh_.advertise<std_msgs::UInt8>("/sentry/can_free_revive", 1);
        pub_can_instant_revive_ = nh_.advertise<std_msgs::UInt8>("/sentry/can_instant_revive", 1);
        pub_operator_input_ = nh_.advertise<geometry_msgs::Point>("/operator/input", 1);
        pub_hurt_info_ = nh_.advertise<std_msgs::UInt8>("/referee/hurt_info", 1);
        
        // ===== Subscribers =====
        sub_cmd_vel_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,
                                                          &MCUCommunicator::cmdVelCallback, this);
        sub_dstar_status_ = nh_.subscribe<std_msgs::Bool>("/dstar_status", 1,
                                                         &MCUCommunicator::dstarStatusCallback, this);
        sub_motion_ = nh_.subscribe<std_msgs::UInt8>("/motion", 1,
                                                     &MCUCommunicator::motionCallback, this);
        sub_recover_ = nh_.subscribe<std_msgs::UInt8>("/recover", 1,
                                                      &MCUCommunicator::recoverCallback, this);
        sub_bullet_up_ = nh_.subscribe<std_msgs::UInt8>("/bullet_up", 1,
                                                        &MCUCommunicator::bulletUpCallback, this);
        sub_bullet_num_ = nh_.subscribe<std_msgs::UInt8>("/bullet_num", 1,
                                                         &MCUCommunicator::bulletNumCallback, this);
        
        // 50 Hz timer for navigation TX
        navigation_timer_ = nh_.createTimer(ros::Duration(nav_period),
                                            &MCUCommunicator::navigationTimerCallback, this);
        
        try
        {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(serial_baudrate_);
            serial_.setBytesize(serial::eightbits);
            serial_.setParity(serial::parity_none);
            serial_.setStopbits(serial::stopbits_one);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
            
            if (serial_.isOpen())
                ROS_INFO("MCU Serial port opened: %s @ %d baud", serial_port_.c_str(), serial_baudrate_);
            else
                ROS_WARN("Failed to open serial port: %s", serial_port_.c_str());
        }
        catch (const std::exception& e)
        {
            ROS_WARN("Serial exception: %s — node will run without MCU data", e.what());
        }
        
        recv_thread_ = std::thread(&MCUCommunicator::receiveThread, this);
    }
    
    ~MCUCommunicator()
    {
        if (recv_thread_.joinable()) recv_thread_.join();
        if (serial_.isOpen()) serial_.close();
    }
    
private:
    ros::NodeHandle nh_;
    serial::Serial serial_;
    std::string serial_port_;
    int serial_baudrate_;
    
    // Publishers: game data from MCU
    ros::Publisher pub_yaw_angle_;
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
    ros::Publisher pub_enemy_hero_;
    ros::Publisher pub_enemy_engineer_;
    ros::Publisher pub_enemy_standard_3_;
    ros::Publisher pub_enemy_standard_4_;
    ros::Publisher pub_enemy_sentry_;
    ros::Publisher pub_suggested_target_;
    ros::Publisher pub_radar_flags_;
    ros::Publisher pub_can_free_revive_;
    ros::Publisher pub_can_instant_revive_;
    ros::Publisher pub_operator_input_;
    ros::Publisher pub_hurt_info_;
    
    ros::Subscriber sub_cmd_vel_;
    ros::Subscriber sub_dstar_status_;
    ros::Subscriber sub_motion_;
    ros::Subscriber sub_recover_;
    ros::Subscriber sub_bullet_up_;
    ros::Subscriber sub_bullet_num_;
    ros::Timer navigation_timer_;
    
    // RX buffer — HK protocol (78 bytes per game data frame)
    uint8_t frame_buffer_[MCU_FRAME_SIZE];
    size_t frame_buffer_index_;
    std::thread recv_thread_;
    
    // Motion command state
    uint8_t current_hp_up_ = 0;
    uint8_t current_bullet_up_ = 0;
    uint8_t current_bullet_num_ = 0;
    uint8_t current_motion_mode_ = 0;
    
    // Navigation state (world-frame vx/vy in m/s, wz in rad/s)
    float current_nav_vx_ = 0.0f;
    float current_nav_vy_ = 0.0f;
    float current_nav_wz_ = 0.0f;
    
    // heroes_never_die: 0=don't revive, 1=confirm free revive, 2=exchange instant revive
    uint8_t current_heroes_never_die_ = 0;
    
    // TX packet sequence counter
    uint8_t nav_packet_seq_ = 0;
    
    // ───── Callbacks ─────
    
    void motionCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        sendMotionCommand(msg->data);
    }
    
    void recoverCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        // heroes_never_die: 0=don't revive, 1=confirm free revive, 2=exchange instant revive
        // Value is sent in every NavigationCommandFrame at 50Hz
        current_heroes_never_die_ = msg->data;
        ROS_INFO("heroes_never_die set to %u", current_heroes_never_die_);
    }
    
    void bulletUpCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_bullet_up_ = (msg->data != 0) ? 1 : 0;
        sendMotionCommand(current_motion_mode_);
    }
    
    void bulletNumCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_bullet_num_ = msg->data;
        sendMotionCommand(current_motion_mode_);
    }
    
    void dstarStatusCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        (void)msg;  // arrival info no longer sent in NavigationFrame
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        current_nav_vx_ = msg->linear.x;   // gimbal-frame vx (m/s)
        current_nav_vy_ = msg->linear.y;   // gimbal-frame vy (m/s)
        current_nav_wz_ = msg->angular.z;  // angular velocity (rad/s)
        ROS_DEBUG("CmdVel: vx=%.4f vy=%.4f wz=%.4f", current_nav_vx_, current_nav_vy_, current_nav_wz_);
    }
    
    void navigationTimerCallback(const ros::TimerEvent& event)
    {
        sendNavigationCommand(current_nav_vx_, current_nav_vy_, current_nav_wz_);
    }
    
    // ───── TX: Navigation command (HK protocol, 21 bytes) ─────
    //
    // Converts float m/s → int16 mm/s, float rad/s → int16 0.01rad/s
    // Packs into HK-framed NavigationCommandFrame with CRC8/CRC16.
    
    void sendNavigationCommand(float vx_ms, float vy_ms, float wz_rads)
    {
        NavigationCommandFrame frame;
        
        // ── Header (9 bytes) ──
        frame.header.sof[0] = HK_FRAME_SOF_H;       // 'H' (0x48)
        frame.header.sof[1] = HK_FRAME_SOF_K;       // 'K' (0x4B)
        frame.header.length = sizeof(NavigationCommandFrame);  // 21 (LE)
        frame.header.packet_type = HK_PACKET_TYPE_NAV; // 0x02
        frame.header.reserved = 0;
        frame.header.packet_seq = nav_packet_seq_++; // 0-255 cycling
        frame.header.reserved2 = 0;
        // CRC8 over bytes 0-7 (header minus crc8 field)
        frame.header.header_crc8 = calculateCRC8((uint8_t*)&frame.header, 8, 0xFF);
        
        // ── Data (8 bytes) ──
        frame.data.heroes_never_die = current_heroes_never_die_;
        frame.data.reserved = 0;
        
        // Unit conversion: m/s → mm/s, clamp to int16 range
        int32_t vx_mm = (int32_t)(vx_ms * 1000.0f);
        int32_t vy_mm = (int32_t)(vy_ms * 1000.0f);
        vx_mm = std::max((int32_t)-32768, std::min((int32_t)32767, vx_mm));
        vy_mm = std::max((int32_t)-32768, std::min((int32_t)32767, vy_mm));
        frame.data.vx = (int16_t)vx_mm;  // mm/s, 右为正
        frame.data.vy = (int16_t)vy_mm;  // mm/s, 前为正
        
        // Unit conversion: rad/s → 0.01 rad/s, clamp to int16
        int32_t wz_centirads = (int32_t)(wz_rads * 100.0f);
        wz_centirads = std::max((int32_t)-32768, std::min((int32_t)32767, wz_centirads));
        frame.data.wz = (int16_t)wz_centirads;  // 0.01 rad/s, 逆时针为正
        
        // ── CRC16 over bytes 0-16 (header + data, 17 bytes) ──
        frame.packet_crc16 = calculateCRC16((uint8_t*)&frame,
                                            sizeof(NavigationCommandFrame) - 4, 0xFFFF);
        
        // ── Trailer ──
        frame.trailer[0] = HK_FRAME_TRAILER_K;  // 'K'
        frame.trailer[1] = HK_FRAME_TRAILER_H;  // 'H'
        
        try
        {
            if (!serial_.isOpen())
            {
                ROS_ERROR_THROTTLE(5, "Serial CLOSED — cannot send nav command");
                return;
            }
            serial_.write((uint8_t*)&frame, sizeof(frame));
            ROS_DEBUG("NavCmd TX: vx=%d vy=%d wz=%d mm/s revive=%u seq=%u",
                      frame.data.vx, frame.data.vy, frame.data.wz,
                      frame.data.heroes_never_die, frame.header.packet_seq);
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Serial nav write exception: %s", e.what());
        }
    }
    
    // ───── TX: Motion command (unchanged, 7 bytes) ─────
    
    void sendMotionCommand(uint8_t motion_mode)
    {
        current_motion_mode_ = motion_mode;
        
        MotionCommandFrame frame;
        frame.sof = MOTION_FRAME_SOF;
        frame.motion_mode_up = motion_mode;
        frame.hp_up = current_hp_up_;
        frame.bullet_up = current_bullet_up_;
        frame.bullet_num = current_bullet_num_;
        frame.eof = MCU_FRAME_EOF;
        frame.crc8 = calculateCRC8((uint8_t*)&frame.sof,
                                   sizeof(MotionCommandFrame) - 2, 0xFF);
        
        try
        {
            if (!serial_.isOpen())
            {
                ROS_ERROR_THROTTLE(5, "Serial CLOSED — cannot send motion command");
                return;
            }
            serial_.write((uint8_t*)&frame, sizeof(frame));
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Serial motion write exception: %s", e.what());
        }
    }
    
    // ───── RX: HK protocol game data frames (78 bytes) ─────
    
    void receiveThread()
    {
        ros::Rate loop_rate(200);
        
        while (ros::ok())
        {
            if (!serial_.isOpen())
            {
                try { serial_.open(); ROS_INFO("Reconnected to serial port"); }
                catch (...) { ros::Duration(2.0).sleep(); continue; }
            }
            
            try
            {
                size_t avail = serial_.available();
                if (avail > 0)
                {
                    std::string raw = serial_.read(avail);
                    std::vector<uint8_t> data(raw.begin(), raw.end());
                    processReceivedData(data);
                }
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("Serial read exception: %s", e.what());
                try { if (serial_.isOpen()) serial_.close(); } catch (...) {}
                frame_buffer_index_ = 0;
            }
            
            loop_rate.sleep();
        }
    }
    
    void processReceivedData(const std::vector<uint8_t>& data)
    {
        for (uint8_t byte : data)
        {
            // Look for SOF 'H' (0x48)
            if (frame_buffer_index_ == 0)
            {
                if (byte == HK_FRAME_SOF_H)
                    frame_buffer_[frame_buffer_index_++] = byte;
                continue;
            }
            
            // Verify second SOF byte 'K' (0x4B)
            if (frame_buffer_index_ == 1)
            {
                if (byte == HK_FRAME_SOF_K)
                {
                    frame_buffer_[frame_buffer_index_++] = byte;
                }
                else
                {
                    frame_buffer_index_ = 0;
                    if (byte == HK_FRAME_SOF_H)
                        frame_buffer_[frame_buffer_index_++] = byte;
                }
                continue;
            }
            
            frame_buffer_[frame_buffer_index_++] = byte;
            
            // Check for complete frame (78 bytes)
            if (frame_buffer_index_ == HK_FRAME_SIZE)
            {
                // Verify trailer 'K', 'H'
                if (frame_buffer_[HK_FRAME_SIZE - 2] == HK_FRAME_TRAILER_K &&
                    frame_buffer_[HK_FRAME_SIZE - 1] == HK_FRAME_TRAILER_H)
                {
                    parseAndPublish();
                }
                else
                {
                    ROS_DEBUG("Invalid trailer: 0x%02X 0x%02X",
                              frame_buffer_[HK_FRAME_SIZE - 2],
                              frame_buffer_[HK_FRAME_SIZE - 1]);
                    
                    // Try to resync: find next 'H','K' in buffer
                    bool found = false;
                    for (size_t i = 1; i < HK_FRAME_SIZE; i++)
                    {
                        if (frame_buffer_[i] == HK_FRAME_SOF_H &&
                            i + 1 < HK_FRAME_SIZE &&
                            frame_buffer_[i + 1] == HK_FRAME_SOF_K)
                        {
                            memmove(frame_buffer_, frame_buffer_ + i, HK_FRAME_SIZE - i);
                            frame_buffer_index_ = HK_FRAME_SIZE - i;
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                        frame_buffer_index_ = 0;
                }
                
                if (frame_buffer_[HK_FRAME_SIZE - 2] == HK_FRAME_TRAILER_K &&
                    frame_buffer_[HK_FRAME_SIZE - 1] == HK_FRAME_TRAILER_H)
                {
                    frame_buffer_index_ = 0;
                }
            }
        }
    }
    
    bool verifyCRC16(MCUDataFrame* frame)
    {
        uint16_t calculated = calculateCRC16((uint8_t*)&frame->header,
                                             HK_FRAME_HEADER_SIZE + HK_FRAME_DATA_SIZE,
                                             0xFFFF);
        if (frame->packet_crc16 != calculated)
        {
            ROS_WARN("CRC16 mismatch: recv=0x%04X calc=0x%04X",
                     frame->packet_crc16, calculated);
            return false;
        }
        return true;
    }
    
    void parseAndPublish()
    {
        MCUDataFrame frame;
        memcpy(&frame, frame_buffer_, HK_FRAME_SIZE);
        
        if (frame.header.packet_type != HK_PACKET_TYPE_GAME)
        {
            ROS_DEBUG("Non-game packet type: 0x%02X", frame.header.packet_type);
            return;
        }
        
        if (!verifyCRC16(&frame))
            return;
        
        // Publish all game data fields
        std_msgs::UInt8 msg_u8;
        std_msgs::UInt16 msg_u16;
        geometry_msgs::Point pos;
        
        msg_u8.data = frame.data.game_progress;
        pub_game_progress_.publish(msg_u8);
        
        msg_u16.data = frame.data.self_hp;
        pub_remain_hp_.publish(msg_u16);
        pub_self_hp_.publish(msg_u16);
        
        msg_u16.data = frame.data.self_max_hp;
        pub_self_max_hp_.publish(msg_u16);
        
        msg_u16.data = frame.data.bullet_remain;
        pub_bullet_remain_.publish(msg_u16);
        
        msg_u8.data = frame.data.occupy_status;
        pub_occupy_status_.publish(msg_u8);
        
        msg_u8.data = frame.data.robot_id;
        pub_robot_id_.publish(msg_u8);
        
        msg_u8.data = frame.data.robot_color;
        pub_robot_color_.publish(msg_u8);
        
        // HP
        msg_u16.data = frame.data.red_1_hp;  pub_red_1_hp_.publish(msg_u16);
        msg_u16.data = frame.data.red_3_hp;  pub_red_3_hp_.publish(msg_u16);
        msg_u16.data = frame.data.red_7_hp;  pub_red_7_hp_.publish(msg_u16);
        msg_u16.data = frame.data.blue_1_hp; pub_blue_1_hp_.publish(msg_u16);
        msg_u16.data = frame.data.blue_3_hp; pub_blue_3_hp_.publish(msg_u16);
        msg_u16.data = frame.data.blue_7_hp; pub_blue_7_hp_.publish(msg_u16);
        msg_u16.data = frame.data.red_dead_bits;  pub_red_dead_.publish(msg_u16);
        msg_u16.data = frame.data.blue_dead_bits; pub_blue_dead_.publish(msg_u16);
        
        // Enemy positions (cm → m)
        pos.x = frame.data.enemy_hero_x / 100.0;
        pos.y = frame.data.enemy_hero_y / 100.0;
        pos.z = 0;
        pub_enemy_hero_.publish(pos);
        
        pos.x = frame.data.enemy_engineer_x / 100.0;
        pos.y = frame.data.enemy_engineer_y / 100.0;
        pub_enemy_engineer_.publish(pos);
        
        pos.x = frame.data.enemy_std3_x / 100.0;
        pos.y = frame.data.enemy_std3_y / 100.0;
        pub_enemy_standard_3_.publish(pos);
        
        pos.x = frame.data.enemy_std4_x / 100.0;
        pos.y = frame.data.enemy_std4_y / 100.0;
        pub_enemy_standard_4_.publish(pos);
        
        pos.x = frame.data.enemy_sentry_x / 100.0;
        pos.y = frame.data.enemy_sentry_y / 100.0;
        pub_enemy_sentry_.publish(pos);
        
        msg_u8.data = frame.data.suggested_target;
        pub_suggested_target_.publish(msg_u8);
        
        msg_u16.data = frame.data.radar_flags;
        pub_radar_flags_.publish(msg_u16);
        
        msg_u8.data = frame.data.can_free_revive;
        pub_can_free_revive_.publish(msg_u8);
        
        msg_u8.data = frame.data.can_instant_revive;
        pub_can_instant_revive_.publish(msg_u8);
        
        // Operator input
        pos.x = frame.data.operator_x;
        pos.y = frame.data.operator_y;
        pos.z = 0;
        pub_operator_input_.publish(pos);
        
        msg_u8.data = frame.data.hurt_info;
        pub_hurt_info_.publish(msg_u8);
        
        ROS_DEBUG("HK frame: game=%u hp=%u bullets=%u",
                  frame.data.game_progress, frame.data.self_hp,
                  frame.data.bullet_remain);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mcu_communicator");
    
    try
    {
        MCUCommunicator comm;
        ROS_INFO("MCU Communicator node started (HK protocol: 21B nav TX, 78B game RX)");
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Fatal error: %s", e.what());
        return 1;
    }
    
    return 0;
}
