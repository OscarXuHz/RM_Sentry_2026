# RoboMaster2023 哨兵ROS消息定义

## 结构设计
1. 本Package用于定义所有模块的**自定义**msg接口和对应服务
2. 根据不同文件夹区分对应模块的msg：
    - detection为感知/全局视野发送消息
    - control为电控底层接收的消息，包括云台和底盘,发布源来自于规划和辅瞄
    - referee_system为裁判系统相关数据，电控只做转发
    - decision为决策发布数据
