#ifndef MCU_COMM_HPP
#define MCU_COMM_HPP

#include <cstdint>
#include <cstring>

// 下位机数据帧结构
struct MCUDataFrame
{
    uint8_t  sof;            // 0x91
    float    yaw_angle;      // 云台yaw弧度
    float    chassis_imu;    // 底盘IMU弧度
    uint8_t  motion_mode;    // 运动模式
    float    operator_x;     // 操作手x
    float    operator_y;     // 操作手y
    uint8_t  robot_id;       // 机器人ID
    uint8_t  robot_color;    // 颜色 (0=红, 1=蓝)
    uint8_t  game_progress;  // 比赛阶段
    uint16_t red_1_hp;       // 红英雄
    uint16_t red_3_hp;       // 红步兵3
    uint16_t red_7_hp;       // 红哨兵
    uint16_t blue_1_hp;      // 蓝英雄
    uint16_t blue_3_hp;      // 蓝步兵3
    uint16_t blue_7_hp;      // 蓝哨兵
    uint16_t red_dead;       // 红方死亡位（bit0英雄，bit2步兵3，bit4哨兵）
    uint16_t blue_dead;      // 蓝方死亡位
    uint16_t self_hp;        // 自身血量
    uint16_t self_max_hp;    // 最大血量
    uint16_t bullet_remain;  // 剩余弹量
    uint8_t  occupy_status;  // 占领状态
    
    float    enemy_hero_x;       // 敌方英雄 X
    float    enemy_hero_y;       // 敌方英雄 Y
    float    enemy_engineer_x;   // 敌方工程 X
    float    enemy_engineer_y;   // 敌方工程 Y
    float    enemy_standard_3_x; // 敌方步兵3 X
    float    enemy_standard_3_y; // 敌方步兵3 Y
    float    enemy_standard_4_x; // 敌方步兵4 X
    float    enemy_standard_4_y; // 敌方步兵4 Y
    float    enemy_sentry_x;     // 敌方哨兵 X
    float    enemy_sentry_y;     // 敌方哨兵 Y
    uint8_t  suggested_target;   // 建议目标
    uint16_t radar_flags;        // 目标选择标志

    uint8_t  crc8;           // CRC8
    uint8_t  eof;            // 0xFE
} __attribute__((packed));

static_assert(sizeof(MCUDataFrame) == 89, "MCUDataFrame must be exactly 89 bytes (must match STM32 nuc_tx_t)");

// 上位机发送的Motion命令帧结构 (共7字节)
struct MotionCommandFrame
{
    uint8_t  sof;              // 0x92 (字节0)
    uint8_t  motion_mode_up;   // 运动模式 (！！这里和实际要和裁判系统通讯的略有不同 0进攻； 1防御； 2移动； 3制动)
    uint8_t  hp_up;            // 0不回血 1回血 
    uint8_t  bullet_up;        // 0不买弹 1买弹 
    uint8_t  bullet_num;       // 买多少 
    uint8_t  crc8;             // CRC8 
    uint8_t  eof;              // 0xFE 
} __attribute__((packed));

static_assert(sizeof(MotionCommandFrame) == 7, "MotionCommandFrame must be exactly 7 bytes");

// 导航数据帧结构 (共17字节)
struct NavigationFrame
{
    uint8_t  sof;            // 0x93 (字节0)
    float    vx;             // (4字节)
    float    vy;             
    float    z_angle;        
    uint8_t  received;      
    uint8_t  arrived;       
    uint8_t  crc8;           // CRC8 
    uint8_t  eof;            // 0xFE
} __attribute__((packed));

static_assert(sizeof(NavigationFrame) == 17, "NavigationFrame must be exactly 17 bytes");

#define MCU_FRAME_SOF 0x91          // 下位机数据帧头
#define MOTION_FRAME_SOF 0x92       // 上位机motion命令帧头
#define NAVIGATION_FRAME_SOF 0x93   // 导航数据帧头
#define MCU_FRAME_EOF 0xFE
#define MCU_FRAME_SIZE sizeof(MCUDataFrame)
#define MOTION_FRAME_SIZE 7
#define NAVIGATION_FRAME_SIZE 17

#endif // MCU_COMM_HPP
