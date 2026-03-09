# 哈尔滨工业大学2024--哨兵导航开源

哈尔滨工业大学I Hiter战队2024赛季哨兵导航开源

## 1. 简介

本套框架基于哈工大I Hiter战队2024赛季哨兵导航程序框架，本款家基于ROS1 noetic开发，整体方案基于哨兵前视雷达+后视深度相机进行局部障碍物感知，机载计算法平台为NUC11，CPU为Intel i7-1165G7.

<img src="IMG/robot.png" alt="image" style="zoom: 67%;" />

## 2. 环境配置(仿真环境)

**注意该代码框架仅支持ROS1，ROS2环境需自行适配！**

总体运行环境基于**Ubuntu20.04 + ROS1 noetic**，简易仿真环境基于Gazebo，下面是环境配置：

### **仿真环境处理**：

```shell
# git拉取代码并建立ROS工作空间后，首先配置部分ros包(如果非noetic请对应自己的ros版本进行更改！)
sudo apt-get install ros-noetic-joint-state-publisher-gui
sudo apt-get install ros-noetic-robot-state-publisher
sudo apt-get install ros-noetic-xacro
sudo apt-get install ros-noetic-pcl-ros
sudo apt-get install ros-noetic-image-transport
sudo apt-get install ros-noetic-camera-info-manager
sudo apt-get install ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-gazebo-ros-pkgs
sudo apt-get install ros-noetic-ros-control
sudo apt-get install ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-pcl-ros
```



```shell
# 安装sophus
git clone <https://github.com/strasdat/Sophus.git>
cd Sophus/
mkdir build
cd build
cmake ..
make
sudo make install

# 安装fmt
cd fmt
mkdir build
cd build
cmake ..
make
sudo make install
# 安装spdlog
sudo apt-get install spdlog
# 或者源码编译
git clone <https://github.com/gabime/spdlog.git>
cd spdlog
mkdir build
cmake ..
make -j123
#源码编译碰到了下面的问题：
#/usr/bin/ld: /usr/local/lib/libspdlog.a(spdlog.cpp.o): relocation 
# R_X86_64_TPOFF32 against `_ZGVZN6spdlog7details2os9thread_idEvE3tid' 
# can not be used when making a shared object; recompile with -fPIC 
# 这个问题看起来是在编译一个共享库时，链接了一个静态库libspdlog.a，
# 但是这个静态库没有使用-fPIC选项编译，导致重定位错误1。你可以尝试重新编译libspdlog.a，
# 加上-fPIC选项，或者使用动态库libspdlog.so2。 这里在spdlog的cmake中添加：
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
```

**仿真环境使用**：

```bash
# 拉到本地工作空间后
catkin_make -j12 # 编译不过就一直编译，因为依赖关系有点复杂
```

**启动两轮小车**

```bash
roslaunch mbot_gazebo mbot_3d_lidar_gazebo.launch
```

**启动控制节点**

```bash
roslaunch mbot_control mbot_control.launch
```

如果想手动开车还有个粗略的开车脚本

```bash
roslaunch mbot_teleop mbot_teleop.launch
```



#### 规划控制节点

1. 规划节点无额外包需要支持，如果不需要运行MPC等局部规划节点可自行将sentry_planning/trajectory_tracking删除
2. 如需运行MPC局部规划节点/完整代码，需安装OCS2，注意我们不是官方安装方式，很多包并不是我们需要的

```shell
# 配置ocs2环境(NMPC求解器)，感兴趣的同学可以对照他的源码和官方文档进行学习
# https://github.com/leggedrobotics/ocs2.git
# https://github.com/leggedrobotics/ocs2_robotic_assets.git  # ocs2自带的mpc控制示例

# Eigen Boost C++(这里我记得安装ros的时候会自己安装，没有的话直接自己搜apt安装很简单)
# ocs2 本仓库安装方式，注意ros包环境
sudo apt install libglpk-dev
sudo apt-get install catkin # 这个默认ros有
sudo apt install ros-noetic-pybind11-catkin
sudo apt install python3-catkin-tools
sudo apt install doxygen doxygen-latex
sudo apt-get install ros-noetic-rqt-multiplot
sudo apt install ros-noetic-grid-map-msgs

catkin_make -j12 # 编译不过就一直编译，因为msg依赖关系有点复杂,一起编译
roslaunch trajectory_generation global_searcher_sim.launch # 全局规划节点
roslaunch tracking_node trajectory_planning_sim.launch # 局部规划局节点

# 如果控制节点无法正常启动，需要手动安装ros control
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers

在 rviz 中选择 3DGoal 工具，鼠标左键点击地图中空白区域以设定目标位置（按G快速选择该工具）
```

所有参数都在launch文件中可调，**注意代码中可能存在逆天的全局路径注意更改**

**关于地图**

1. 本地图基于RMUC2024，地图文件在trajectory_generation/map中，如果想要过桥洞版本的请将参数表中指定的地图文件更改为过桥洞版本(全局规划trajectory_generation与局部规划trajectory_tracking都要更改)

   过桥洞版本参数文件

   ```c++
       <param name="trajectory_generator/occ_file_path" type="string" value="$(find trajectory_generation)/map/occ2024low.png"/>
   ```

   非过桥洞参数文件

   ```c++
       <param name="trajectory_generator/occ_file_path" type="string" value="$(find trajectory_generation)/map/occ2024.png"/>
   ```

 2. 如果需要更改地图，需将占用栅格地图(occ), 高度梯度(bev)， 与车道中心线地图(Topo) 都进行更改，可基于雷达建图的点云写处理脚本与骨架提取自行生成对应的三个二位地图，并更改对应的地图参数，如地图的尺寸，高度偏置参数

    ```
        <param name="trajectory_generator/map_x_size" value="30.0"/>
        <param name="trajectory_generator/map_y_size" value="18.0"/>  <!-- 仿真环境参数-->
            <param name="trajectory_generator/height_bias" value="-0.5"/>  <!-- 地图高度偏置参数-->
        <param name="trajectory_generator/height_interval" value="2.0"/> <!-- 地图高度间隔参数,高度地图所有像素的高度间隔-->
        <param name="trajectory_generator/height_threshold" value="0.1"/> <!-- 地图高度阈值参数,障碍物点云高度检测阈值-->
        <param name="trajectory_generator/height_sencond_high_threshold" value="0.3"/> <!-- 桥梁地图高度阈值参数,这个是顶部高度阈值-->
    ```



### 运行

配置好环境后，程序运行顺序如下：

```
roslaunch mbot_gazebo mbot_3d_lidar_gazebo.launch  # 启动简易仿真
roslaunch mbot_control mbot_control.launch  # 启动控制节点
roslaunch trajectory_generation global_searcher_sim.launch # 全局规划节点
roslaunch tracking_node trajectory_planning_sim.launch # 局部规划局节点
```

会出现如下程序运行画面：

<img src="IMG/程序运行.png" alt="image" style="zoom: 80%;" />



## 3. 方案设计

2024/2025赛季比赛地图复杂，且存在飞坡，台阶，斜坡，桥洞等多种地形需要处理，且需要执行多种不同任务，因此采用2.5维的地图感知及规划方案，在地图搜索的过程中就能处理复杂地形，因此我们的规划分为四部分：**地图处理与局部障碍感知，全局路径搜索，全局轨迹优化，局部跟踪避障**。

**全局路径搜索：**

<img src="IMG/topo搜索.png" alt="image" style="zoom: 80%;" />

**轨迹优化：**

<img src="IMG/轨迹优化.png" alt="image" style="zoom: 80%;" />

**NMPC局部轨迹跟踪+避障：**

<img src="IMG/NMPC流程.png" alt="image" style="zoom: 80%;" />

**重规划：**

<img src="IMG/重规划流程.png" alt="image" style="zoom: 80%;" />



# 详细技术方案见技术文档