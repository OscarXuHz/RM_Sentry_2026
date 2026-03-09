# MCU通讯模块 - 快速参考

## 文件结构
```
decision_node/
├── include/decision_node/
│   └── mcu_comm.hpp                          # MCU数据结构定义
├── src/
│   ├── mcu_communicator.cpp                  # 主通讯节点
│   ├── mcu_simulator.cpp                     # 测试模拟器
│   └── [其他已有文件...]
├── launch/
│   └── mcu_communicator.launch              # 启动配置
├── CMakeLists.txt                           # (已修改)
├── package.xml                              # (已修改)
├── MCU_COMMUNICATOR_README.md               # 详细文档
├── MCU_TEST_GUIDE.md                        # 测试指南
└── IMPLEMENTATION_SUMMARY.md                # 实现总结
```

## 一键编译
```bash
cd ~/decision_ws
catkin_make
```

## 一键测试（使用模拟器）
```bash
# 启动ROS（终端1）
roscore

# 启动通讯模块（终端2）
source ~/decision_ws/devel/setup.bash
rosrun decision_node mcu_communicator _serial_port:=/dev/ttyUSB0

# 启动模拟器（终端3）
source ~/decision_ws/devel/setup.bash
rosrun decision_node mcu_simulator _serial_port:=/dev/ttyUSB1

# 验证（终端4）
source ~/decision_ws/devel/setup.bash
rostopic list | grep -E "referee|nav|robot"
rostopic echo /referee/game_progress
```

## 与实际下位机连接
```bash
# 查找串口设备
ls /dev/ttyUSB*

# 启动通讯模块（替换串口号）
rosrun decision_node mcu_communicator _serial_port:=/dev/ttyUSB0 _baudrate:=115200
```

## 使用Launch文件
```bash
# 编辑launch文件配置串口
nano ~/decision_ws/src/decision_node/launch/mcu_communicator.launch

# 启动
roslaunch decision_node mcu_communicator.launch
```

## ROS Topics 快速查看

### 已发布的所有Topic
```bash
# 列出所有topic
rostopic list | grep -E "referee|nav|robot"

# 监看数据流
rostopic echo /referee/game_progress
rostopic echo /nav/yaw_angle
rostopic echo /nav/chassis_imu
rostopic echo /robot/self_hp

# 检查频率
rostopic hz /referee/game_progress
```

## 数据帧格式快速参考
| 字节 | 字段 | 类型 | 范围 |
|------|------|------|------|
| 0 | sof | uint8 | 0x91 |
| 1-4 | yaw_angle | float | -π ~ +π |
| 5-8 | chassis_imu | float | -π ~ +π |
| 9 | motion_mode | uint8 | 0-3 |
| 10-13 | operator_x | float | - |
| 14-17 | operator_y | float | - |
| 18 | robot_id | uint8 | - |
| 19 | robot_color | uint8 | 0红/1蓝 |
| 20 | game_progress | uint8 | 0-2 |
| 21-32 | HP值 (6个) | uint16×6 | 0-400 |
| 33-36 | 死亡位 (2个) | uint16×2 | - |
| 37-38 | self_hp | uint16 | 0-400 |
| 39-40 | self_max_hp | uint16 | 0-400 |
| 41-42 | bullet_remain | uint16 | 0-1000 |
| 43 | occupy_status | uint8 | 0-3 |
| 44 | crc8 | uint8 | - |
| 45 | eof | uint8 | 0xFE |

## 故障排查速查表

| 问题 | 解决方案 |
|------|---------|
| 找不到串口 | `ls /dev/tty*` 检查设备 |
| Permission denied | `sudo usermod -a -G dialout $USER` |
| 无法打开串口 | 检查波特率、检查是否被占用 |
| 收不到数据 | 验证帧头/帧尾、检查模拟器是否启动 |
| 数据格式错误 | 检查46字节帧大小、验证字节序 |
| 决策系统收不到 | 确认通讯模块已启动、检查topic名称 |

## 关键代码位置

### 数据结构定义
- 文件: `include/decision_node/mcu_comm.hpp`
- 结构体: `MCUDataFrame` (46字节)

### 接收数据处理
- 文件: `src/mcu_communicator.cpp`
- 方法: `processReceivedData()` - 字节接收和帧同步
- 方法: `parseAndPublish()` - 数据解析和发布

### 模拟数据生成
- 文件: `src/mcu_simulator.cpp`
- 方法: `sendFrame()` - 生成和发送模拟帧

## 集成到决策系统

### 已兼容的Topic（无需修改）
```cpp
// strategy_node.cpp 中已有订阅
sub_game_progress = nh.subscribe<std_msgs::Int32>("/referee/game_progress", ...)
sub_remain_hp = nh.subscribe<std_msgs::Int32>("/referee/remain_hp", ...)
sub_bullet = nh.subscribe<std_msgs::Int32>("/referee/bullet_remain", ...)
sub_occupy_status = nh.subscribe<std_msgs::Int32>("/referee/occupy_status", ...)
```

### 使用新增数据（如需要）
在 `strategy_node.cpp` 中添加：
```cpp
auto sub_yaw = nh.subscribe<std_msgs::Float32>("/nav/yaw_angle", 1,
    [&](const std_msgs::Float32::ConstPtr& msg) {
        // 处理云台角度数据
        yaw_angle_ = msg->data;
    });
```

## 性能指标

| 指标 | 值 |
|------|-----|
| 帧大小 | 46字节 |
| 发送频率 | 10Hz |
| 波特率 | 115200 bps |
| 吞吐量 | 460 B/s (≈3.68 kbps) |
| 接收线程频率 | 100Hz |

## 依赖包

### 新增依赖
- `serial` - ROS串口通讯库

### 编译依赖
- `roscpp`
- `std_msgs`
- `geometry_msgs`
- `behaviortree_cpp_v3` (已有)

## 扩展开发

### 添加CRC校验
```cpp
// 在 mcu_comm.hpp 中添加
uint8_t getCRC8(const uint8_t* buf, size_t len) {
    uint8_t crc = 0xFF;
    // CRC-8算法实现
    return crc;
}
```

### 添加数据验证
```cpp
// 在 parseAndPublish() 中添加
if (frame.self_hp > frame.self_max_hp) {
    ROS_WARN("Invalid HP: %u > %u", frame.self_hp, frame.self_max_hp);
    return;  // 丢弃无效帧
}
```

## 常用命令

```bash
# 编译整个工作空间
catkin_make

# 只编译decision_node
catkin_make --pkg decision_node

# 清除构建结果
catkin_make clean

# 查看编译错误
catkin_make -DCATKIN_ENABLE_TESTING=1

# 运行通讯模块
rosrun decision_node mcu_communicator _serial_port:=/dev/ttyUSB0

# 查看节点信息
rosnode info /mcu_communicator

# 查看发布的topics
rostopic list

# 实时监看数据
rqt_plot /referee/game_progress/data
```

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2024-01-27 | 初始版本，包含完整的通讯功能 |

## 支持

- 详细文档: 见 `MCU_COMMUNICATOR_README.md`
- 测试指南: 见 `MCU_TEST_GUIDE.md`
- 实现详情: 见 `IMPLEMENTATION_SUMMARY.md`

---

**最后更新**: 2024-01-27
**作者**: AI Assistant
**许可证**: BSD
