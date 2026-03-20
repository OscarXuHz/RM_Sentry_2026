# MCU通讯模块 (MCU Communicator)

## 概述
本模块用于接收下位机通过串口发送的数据帧，并将其解析后通过ROS topic发布，供决策系统使用。

## 数据帧结构
下位机发送的数据帧共46字节，结构如下：

```c
struct MCUDataFrame {
    uint8_t  sof;            // 0x91 (字节0)
    float    yaw_angle;      // 云台yaw弧度 (字节1-4)
    float    chassis_imu;    // 底盘IMU弧度 (字节5-8)
    uint8_t  motion_mode;    // 运动模式 (字节9)
    float    operator_x;     // 操作手x (字节10-13)
    float    operator_y;     // 操作手y (字节14-17)
    uint8_t  robot_id;       // 机器人ID (字节18)
    uint8_t  robot_color;    // 颜色 (字节19)
    uint8_t  game_progress;  // 比赛阶段 (字节20)
    uint16_t red_1_hp;       // 红英雄 (字节21-22)
    uint16_t red_3_hp;       // 红步兵3 (字节23-24)
    uint16_t red_7_hp;       // 红哨兵 (字节25-26)
    uint16_t blue_1_hp;      // 蓝英雄 (字节27-28)
    uint16_t blue_3_hp;      // 蓝步兵3 (字节29-30)
    uint16_t blue_7_hp;      // 蓝哨兵 (字节31-32)
    uint16_t red_dead;       // 红方死亡位 (字节33-34)
    uint16_t blue_dead;      // 蓝方死亡位 (字节35-36)
    uint16_t self_hp;        // 自身血量 (字节37-38)
    uint16_t self_max_hp;    // 最大血量 (字节39-40)
    uint16_t bullet_remain;  // 剩余弹量 (字节41-42)
    uint8_t  occupy_status;  // 占领状态 (字节43)
    uint8_t  crc8;           // CRC8 (字节44)
    uint8_t  eof;            // 0xFE (字节45)
}
```

## 发布的ROS Topics

### 现有Topic（与decision_node兼容）
- `/referee/game_progress` (std_msgs/Int32) - 比赛进度
- `/referee/remain_hp` (std_msgs/Int32) - 自身血量
- `/referee/bullet_remain` (std_msgs/Int32) - 剩余弹量
- `/referee/occupy_status` (std_msgs/Int32) - 占领状态

### 新增Topic - 导航相关
- `/nav/yaw_angle` (std_msgs/Float32) - 云台yaw角度(弧度)
- `/nav/chassis_imu` (std_msgs/Float32) - 底盘IMU角度(弧度)
- `/nav/operator_position` (geometry_msgs/Vector3) - 操作手位置 (x, y, z)
- `/nav/motion_mode` (std_msgs/UInt8) - 运动模式

### 新增Topic - 机器人状态
- `/robot/robot_id` (std_msgs/UInt8) - 机器人ID
- `/robot/robot_color` (std_msgs/UInt8) - 机器人颜色 (0=红, 1=蓝)
- `/robot/self_hp` (std_msgs/UInt16) - 自身血量(UInt16版本)
- `/robot/self_max_hp` (std_msgs/UInt16) - 最大血量

### 新增Topic - 其他机器人血量
- `/referee/red_1_hp` (std_msgs/UInt16) - 红方1号(英雄)血量
- `/referee/red_3_hp` (std_msgs/UInt16) - 红方3号(步兵)血量
- `/referee/red_7_hp` (std_msgs/UInt16) - 红方7号(哨兵)血量
- `/referee/blue_1_hp` (std_msgs/UInt16) - 蓝方1号(英雄)血量
- `/referee/blue_3_hp` (std_msgs/UInt16) - 蓝方3号(步兵)血量
- `/referee/blue_7_hp` (std_msgs/UInt16) - 蓝方7号(哨兵)血量

### 新增Topic - 死亡状态
- `/referee/red_dead` (std_msgs/UInt16) - 红方死亡位图
  - bit 0: 1号(英雄)死亡
  - bit 2: 3号(步兵)死亡
  - bit 4: 7号(哨兵)死亡
- `/referee/blue_dead` (std_msgs/UInt16) - 蓝方死亡位图
  - bit 0: 1号(英雄)死亡
  - bit 2: 3号(步兵)死亡
  - bit 4: 7号(哨兵)死亡

## 编译和运行

### 编译
```bash
cd ~/decision_ws
catkin_make
```

### 运行
方法1: 使用launch文件
```bash
roslaunch decision_node mcu_communicator.launch
```

方法2: 直接运行节点（修改串口号）
```bash
rosrun decision_node mcu_communicator _serial_port:=/dev/ttyUSB0 _baudrate:=115200
```

## 参数配置

在launch文件中配置以下参数：

```xml
<param name="serial_port" value="/dev/ttyUSB0" />  <!-- 串口号 -->
<param name="baudrate" value="115200" />            <!-- 波特率 -->
```

或通过命令行参数：
```bash
rosrun decision_node mcu_communicator _serial_port:=/dev/ttyUSB0 _baudrate:=115200
```

## 测试

### 查看发布的topic
```bash
# 列出所有topic
rostopic list | grep referee
rostopic list | grep nav
rostopic list | grep robot

# 查看具体topic数据
rostopic echo /referee/game_progress
rostopic echo /nav/yaw_angle
```

### 测试频率
```bash
rostopic hz /referee/game_progress
```

## 故障排查

1. **找不到串口设备**
   - 检查设备是否连接
   - 使用 `ls /dev/tty*` 查找串口
   - 检查用户权限: `sudo usermod -a -G dialout $USER`

2. **无法打开串口**
   - 确认波特率是否正确
   - 检查是否有其他程序占用串口

3. **未收到数据**
   - 检查下位机是否在发送数据
   - 验证帧头(0x91)和帧尾(0xFE)
   - 检查波特率设置

4. **数据帧格式错误**
   - 检查数据帧大小(必须是46字节)
   - 验证字节序(little-endian for float/uint16)

## 集成到决策系统

decision_node已自动订阅以下topic（从`strategy_node.cpp`看）：
- `/referee/game_progress`
- `/referee/remain_hp`
- `/referee/bullet_remain`
- `/referee/friendly_score` (需要通讯模块发布)
- `/referee/enemy_score` (需要通讯模块发布)
- `/referee/occupy_status`

如需使用其他数据(如导航数据)，在`strategy_node.cpp`中添加新的subscriber即可。

## 文件说明

- `mcu_comm.hpp` - MCU数据帧结构体定义
- `mcu_communicator.cpp` - 通讯节点主程序
- `mcu_communicator.launch` - Launch文件配置

## 许可证
BSD
