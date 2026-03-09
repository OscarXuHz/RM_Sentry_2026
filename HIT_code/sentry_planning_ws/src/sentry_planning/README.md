## 仓库使用
进入工作区的src目录下
```shell
git clone git@gitee.com:sunjingwenrobot/sentry_planning.git
```
该仓库的根文件夹为“sentry_planning”，与仿真仓库根文件夹“sentry_gazebo_2023”同级别


## 运行代码
### 添加RM场地点云地图文件
* 下载点云地图文件：
* 文件存放目录 planning/global_searcher/map/点云地图文件名.pcd
* planning/global_searcher/cfg/global_searcher.yaml中修改pcd地图文件路径为planning/global_searcher/map/点云地图文件名.pcd

### 单独运行规划模块
* 编译规划模块 catkin_make -DCATKIN_WHITELIST_PACKAGES="global_searcher;rviz_plugins;slaver;waypoint_generator"
* 启动规划模块 roslaunch global_searcher global_searcher.launch
* 在 rviz 中选择 3DGoal 工具，鼠标左键点击地图中空白区域以设定目标位置（按G快速选择该工具）

### 启动导航系统仿真
* roslaunch mbot_gazebo mbot_rmuc_lidar_gazebo.launch 启动仿真环境（启动后调整模型RMchangdi1230的位置为（0，0，0），调整模型mrobot的位置为（5，7.5，0.3））
* roslaunch mbot_control mbot_control.launch 启动控制器
* roslaunch fast_lio mapping_velodyne.launch 启动建图
* roslaunch global_searcher global_searcher.launch 启动路径规划
* roslaunch trajectory_planning trajectory_planning.launch 启动轨迹规划
* 在 rviz 中选择 3DGoal 工具，鼠标左键点击地图中空白区域以设定目标位置（按G快速选择该工具）

## 比赛注意事项

1. 修地图时一定注意各处台阶，高地边缘与可通行地区的高度差是否被噪声影响
2. 修好的地图在仿真测试跑的时候，测试过桥洞，下台阶，以及一定注意是否会从桥洞上桥，上台阶这种东西出现
