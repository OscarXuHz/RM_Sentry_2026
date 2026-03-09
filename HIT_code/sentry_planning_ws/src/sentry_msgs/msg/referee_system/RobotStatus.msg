# 比赛机器人状态 0x0201 10Hz
Header header

uint8 id # 机器人ID
uint8 level  # 机器人等级
uint16 remain_hp  # 剩余血量
uint16 max_hp  # 最大血量
uint16 shooter_barrel_cooling_value  # 枪口冷却速度
uint16 shooter_barrel_heat_limit  # 机器人枪口热量上限
uint16 chassis_power_limit  # 底盘功率上限
# 主控电源输出情况
bool gimbal_output
bool chassis_output
bool shooter_output
