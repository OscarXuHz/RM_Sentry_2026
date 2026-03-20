#include "../include/tracking_manager.h"
#include "../include/RM_GridMap.h"
#include <numeric>
//#include "../include/visualization_utils.h"



void tracking_manager::init(ros::NodeHandle &nh)
{
    nh.param("tracking_node/wheel_tread", robot_wheel_tread, 0.42);
    nh.param("tracking_node/target_point_arrive_radius", target_point_arrive_radius, 0.05);
    nh.param("tracking_node/occ_file_path", occ_file_path, std::string("occfinal.png"));
    nh.param("tracking_node/bev_file_path", bev_file_path, std::string("bevfinal.png"));
    nh.param("tracking_node/distance_map_file_path", distance_map_file_path, std::string("distance.png"));

    nh.param("tracking_node/map_resolution", map_resolution, 0.1);
    nh.param("tracking_node/map_lower_point_x", map_lower_point(0), 0.0);
    nh.param("tracking_node/map_lower_point_y", map_lower_point(1), 0.0);
    nh.param("tracking_node/map_lower_point_z", map_lower_point(2), 0.0);
    nh.param("tracking_node/map_x_size", map_x_size, 28.0);
    nh.param("tracking_node/map_y_size", map_y_size, 15.0);
    nh.param("tracking_node/map_z_size", map_z_size, 2.0);
    nh.param("tracking_node/search_height_min", search_height_min, 0.1);
    nh.param("tracking_node/search_height_max", search_height_max, 1.2);
    nh.param("tracking_node/search_radius", search_radius, 5.0);
    nh.param("tracking_node/robot_radius_dash", robot_radius_dash, 0.1);
    nh.param("tracking_node/robot_radius", robot_radius, 0.3);

    ROS_WARN("map/occ_file_path: %s", occ_file_path.c_str());
    ROS_WARN("map/bev_file_path: %s", bev_file_path.c_str());
    ROS_WARN("map/distance_map_file_path: %s", distance_map_file_path.c_str());


    sentry_wheelyawspeed_feedback = nh.subscribe("/slaver/wheel_state", 2, &tracking_manager::rcvSentryWheelSpeedYawCallback, this);
    gazebo_real_pos_sub = nh.subscribe("/gazebo/model_states", 1, &tracking_manager::rcvGazeboRealPosCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
    lidar_imu_pos_sub = nh.subscribe("/odometry_imu", 1, &tracking_manager::rcvLidarIMUPosCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());

    robot_status_sub = nh.subscribe("/slaver/robot_status", 1, &tracking_manager::rcvRobotStatusCallback, this);
    sentry_HP_sub = nh.subscribe("/slaver/robot_HP", 1, &tracking_manager::rcvSentryHPJudgeCallback, this);

    robot_left_speed_pub = nh.advertise<std_msgs::Float64>("/mbot/left_wheel_joint_controller/command", 1);
    robot_right_speed_pub = nh.advertise<std_msgs::Float64>("/mbot/right_wheel_joint_controller/command", 1);
    sentry_speed_pub = nh.advertise<sentry_msgs::slaver_speed>("/sentry_des_speed", 1);
    m_imu_acc_pub = nh.advertise<std_msgs::Float64>("/imu_filter", 1);
    m_kal_acc_pub = nh.advertise<std_msgs::Float64>("/kal_filter", 1);
    m_solver_status_pub = nh.advertise<std_msgs::Bool>("/solver_status", 1);
    global_replan_pub = nh.advertise<std_msgs::Bool>("/replan_flag", 1);
    robot_cur_yaw_pub = nh.advertise<std_msgs::Float64>("/robot_cur_yaw_reg", 1);

    nh.param("tracking_node/use_omega_output", use_omega_output_, false);
    if (use_omega_output_) {
        ROS_WARN("[tracking_manager] Omega output mode ENABLED: slaver_speed fields = (omega, vy_body, vx_body)");
    }

    map_inv_resolution = 1.0 / map_resolution;
    map_upper_point(0) = map_lower_point(0) + map_x_size;
    map_upper_point(1) = map_lower_point(1) + map_y_size;
    map_upper_point(2) = map_lower_point(2) + map_z_size;

    grid_max_id_x = (int)(map_x_size * map_inv_resolution);
    grid_max_id_y = (int)(map_y_size * map_inv_resolution);
    grid_max_id_z = (int)(map_z_size * map_inv_resolution);

    global_map.reset(new GlobalMap);
    cv::Mat occ_map3c;
    occ_map3c = cv::imread(occ_file_path);
    if (occ_map3c.empty()) {
        ROS_FATAL("[tracking_manager] Failed to load OCC map: %s", occ_file_path.c_str());
        ros::shutdown();
        return;
    }
    std::vector <cv::Mat> channels;
    cv::split(occ_map3c, channels);
    cv::Mat occ_map = channels.at(0);

    global_map->initGridMap(nh, occ_map, bev_file_path, distance_map_file_path, map_resolution,
                            map_lower_point, map_upper_point, grid_max_id_x, grid_max_id_y, grid_max_id_z,
                            robot_radius, search_height_min, search_height_max, search_radius); ///////////////

    localplanner.reset(new LocalPlanner);
    localplanner->init(nh, global_map);

    vislization_util.reset(new Vislization);
    vislization_util->init(nh);

    planningMode = planningType::FASTMOTION;  // 初始为快速移动模式
}

void tracking_manager::gazeboVelAccSmooth(double speed, double dt)  // 对gazebo的速度加速度进行kalman平滑处理
{
    Eigen::Vector2d measure_speed;
    measure_speed << speed, 0;

    if(!gazebo_accvel_filter.m_filter_inited)
    {
        gazebo_accvel_filter.initParam(1e-4, 1e-4, 0.004, false);
        gazebo_accvel_filter.m_filter_inited = true;
        gazebo_accvel_filter.setState(measure_speed);
    }
    else
    {
        gazebo_accvel_filter.predictUpdate(dt);
        gazebo_accvel_filter.measureUpdate(measure_speed);
    }
}

void tracking_manager::velocityYawSmooth(double velocity_yaw, double dt)  // 对速度和yaw进行平滑处理
{
    Eigen::Vector2d measure_speedyaw;
    Eigen::Vector2d speed_yaw_temp;
    measure_speedyaw << velocity_yaw, 0;
    kalman_filter_speed_yaw.getResults(speed_yaw_temp);
    if(measure_speedyaw(0) - speed_yaw_temp(0) > M_PI){
        kalman_filter_speed_yaw.setState(measure_speedyaw);
    }

    if(!kalman_filter_speed_yaw.m_filter_inited)
    {
        kalman_filter_speed_yaw.initParam(tracking_manager::speed_init_r, tracking_manager::speed_init_q, 0.01, false);
        kalman_filter_speed_yaw.m_filter_inited = true;
        kalman_filter_speed_yaw.setState(measure_speedyaw);
    }
    else
    {
        kalman_filter_speed_yaw.predictUpdate(dt, tracking_manager::robot_odometry_speed);
        kalman_filter_speed_yaw.measureUpdate(measure_speedyaw);
    }
}

void tracking_manager::lidarSmooth(Eigen::Vector3d position, Eigen::Vector3d velocity, double dt)
{  // 雷达定位相关反馈数据滤波平滑
    Eigen::Vector2d measurevalue_x;
    Eigen::Vector2d measurevalue_y;
    Eigen::Vector2d measurevalue_yaw;

    measurevalue_x << position(0), 0;
    measurevalue_y << position(1), 0;
    measurevalue_yaw << position(2), velocity(2);
    Eigen::Vector2d yaw_temp;
    kalman_filter_yaw.getResults(yaw_temp);

    if(abs(measurevalue_yaw(0) - yaw_temp(0)) > M_PI )  /// 防止角度跳变
    {
        kalman_filter_yaw.setState(measurevalue_yaw);
    }

    if (!kalman_filter_speedx.m_filter_inited)
    {
        kalman_filter_speedx.initParam(tracking_manager::speed_init_r, tracking_manager::speed_init_q, 0.01, false);
        kalman_filter_speedy.initParam(tracking_manager::speed_init_r, tracking_manager::speed_init_q, 0.01, false);
        kalman_filter_yaw.initParam(tracking_manager::yaw_init_r, tracking_manager::yaw_init_q, 0.01, false);

        ROS_ERROR("start init");
        kalman_filter_speedx.m_filter_inited = true;
        kalman_filter_speedx.setState(measurevalue_x);

        kalman_filter_speedy.m_filter_inited = true;
        kalman_filter_speedy.setState(measurevalue_y);

        kalman_filter_yaw.m_filter_inited = true;
        kalman_filter_yaw.setState(measurevalue_yaw);
    }
    else
    {
        kalman_filter_speedx.predictUpdate(dt);
        kalman_filter_speedx.measureUpdate(measurevalue_x);

        kalman_filter_speedy.predictUpdate(dt);
        kalman_filter_speedy.measureUpdate(measurevalue_y);

        kalman_filter_yaw.predictUpdate(dt);
        kalman_filter_yaw.measureUpdate(measurevalue_yaw);
    }
}


void tracking_manager::rcvSentryHPJudgeCallback(const sentry_msgs::RobotsHPConstPtr &msg)
{
    if (sentryColor == teamColor::red) {
        mate_outpost_hp = msg->red_outpost_hp;
    } else {
        mate_outpost_hp = msg->blue_outpost_hp;
    }
    ROS_WARN("mate_outpost_hp: %f", mate_outpost_hp);

    if (mate_outpost_hp < 1) {
        if(!isxtl){
            replan_now = true;
        }
        isxtl = true;  //  前哨站血量够低的话才可以进小陀螺
    }else{
        isxtl = false;
    }

    countHP ++;   /// 血量返回计数
    int current_HP;
    if(sentryColor == teamColor::red){  // 红蓝方判断
        current_HP = msg->red_sentry_hp;
    }else{
        current_HP = msg->blue_sentry_hp;
    }
    ROS_WARN("[FSM SentryHP] current_HP: %f", current_HP);

    if(sentry_HP == 0){  /// 初始化当前的烧饼血量
        sentry_HP = current_HP;
    }

    if((sentry_HP - current_HP) >= 2){ /// 血量减少超过2，说明被攻击，开始计数并转换标志位
        countHP = 0;
        is_attacked = true;
        ROS_ERROR("[FSM SentryHP] is_attacked!!");
    }
    else if(countHP > 20 && (sentry_HP == current_HP))
    {
        is_attacked = false;
        ROS_WARN("[FSM SentryHP] Safe!!");
    }
    sentry_HP = current_HP;
    return;
}

void tracking_manager::rcvRobotStatusCallback(const sentry_msgs::RobotStatusConstPtr &msg) {

    if (msg->id == 7) {  //! 蓝色107
        sentryColor = teamColor::red;
    } else {
        sentryColor = teamColor::blue;
    }
}


void tracking_manager::rcvSentryWheelSpeedYawCallback(const geometry_msgs::Vector3ConstPtr &state)
{
    has_wheel_state_ = true;
    if(abs(state->x) < 10.0){
        tracking_manager::robot_wheel_speed = state->x;
    }
    if(abs(tracking_manager::robot_wheel_yaw - state->y* M_PI / 180) < 10000){
        tracking_manager::robot_wheel_yaw = state->y * M_PI / 180;
    }
}


void tracking_manager::rcvLidarIMUPosCallback(const nav_msgs::OdometryConstPtr &state)
{
    /* 实时位置 */
    control_time = ros::Time::now(); // 当此控制周期的时间
    double cost_time = (control_time - localplanner->start_tracking_time).toSec();

    /* 实时位置 */
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    pose = state->pose.pose;
    twist = state->twist.twist;

    // Compute LiDAR yaw first so we can correct the reported position to the
    // chassis / gimbal centre. Offset source: aft_mapped→gimbal_frame TF in
    // real_robot_transform.cpp: translation = (-0.011, -0.17166, 0) m in LiDAR frame.
    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;
    double w = pose.orientation.w;
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    double robot_add_yaw = 0.0;  // TODO 这里可能会有点问题，在非陀螺模式下不保证底盘一定在+-180度内，如果出现异常情况的话会疯转
    double robot_lidar_yaw = atan2f(siny_cosp, cosy_cosp);

    // Fix 37: Use raw aft_mapped (LiDAR) position — NO chassis offset.
    // trajectory_generation generates the reference polynomial in aft_mapped
    // coordinates (replan_fsm.cpp uses raw pose.position with no offset).
    // Applying a LiDAR→chassis offset here puts the MPC observation in a
    // different frame from the reference, causing systematic tracking error
    // and the predicted path diverging from the reference path.
    robot_cur_position(0) = pose.position.x;
    robot_cur_position(1) = pose.position.y;
    robot_cur_position(2) = 0.0;

    global_map->odom_position(0) = pose.position.x;
    global_map->odom_position(1) = pose.position.y;
    global_map->odom_position(2) = pose.position.z;

    double robot_x = pose.position.x;
    double robot_y = pose.position.y;

    double robot_yaw_temp;
    if (has_wheel_state_) {
        // Use wheel encoder yaw when available
        robot_yaw_temp = robot_wheel_yaw;
    } else {
        // Fallback: use LiDAR/IMU yaw when no wheel state
        robot_yaw_temp = robot_lidar_yaw;
    }
    int k = robot_yaw_temp / (2 * M_PI);
    robot_yaw_temp = robot_yaw_temp - k * (2 * M_PI);

    int j = robot_yaw_temp / M_PI;
    robot_yaw_temp = robot_yaw_temp - j * (2 * M_PI);
    robot_cur_yaw = robot_yaw_temp;
    robot_add_yaw = (k+j)*(2*M_PI);

    double v_ctrl = 0.0;
    double phi_ctrl = 0.0;
    double acc_ctrl = 0.0;
    double angular_ctrl = 0.0;
    Eigen::Vector4d predict_input = Eigen::Vector4d::Zero();  // init to zero to avoid garbage if solver fails

    double line_speed_temp = sqrt(pow(abs(twist.linear.x), 2) + pow(abs(twist.linear.y), 2));
    if (has_wheel_state_) {
        // Use wheel encoder speed
        robot_cur_speed(0) = robot_wheel_speed * cos(robot_cur_yaw);
        robot_cur_speed(1) = robot_wheel_speed * sin(robot_cur_yaw);
    } else {
        // Fallback: derive speed from odom twist (LiDAR/IMU odometry)
        robot_cur_speed(0) = twist.linear.x;
        robot_cur_speed(1) = twist.linear.y;
    }

    //  判断一下是不是需要加入抵达终点的判断来set速度为0
    double line_speed = sqrt(pow(abs(robot_cur_speed(0)), 2) + pow(abs(robot_cur_speed(1)), 2));

    Eigen::VectorXd sentry_state;
    std::vector<Eigen::Vector4d> state_deque;
    std::vector<Eigen::Vector2d> input_deque;

    sentry_state = Eigen::VectorXd::Zero(4);
    sentry_state << robot_x, robot_y, line_speed, robot_cur_yaw;

    if(localplanner->m_get_global_trajectory){
        arrival_goal = false;
    }
    if(arrival_goal){
        v_ctrl = 0.0;
        phi_ctrl = robot_lidar_yaw;
        acc_ctrl = 0.0;
        angular_ctrl = 0.0;
        Eigen::Vector2d optimal_differential_speed;
        Eigen::Vector4d MPC_Control;

        optimal_differential_speed = getDifferentialModelSpeed(v_ctrl, angular_ctrl, robot_wheel_tread);
        publishMbotOptimalSpeed(optimal_differential_speed);

        if (use_omega_output_) {
            // omega mode (new robot): (ω=0, unused, v=0, mode)
            MPC_Control(0) = 0.0;               // ω = 0 (stopped)
            MPC_Control(1) = 0.0;               // unused
            MPC_Control(2) = 0.0;               // v = 0
        } else {
            // omni mode (old robot): (heading=current, lidar_yaw, v=0, mode)
            MPC_Control(0) = phi_ctrl;           // keep current heading
            MPC_Control(1) = phi_ctrl;           // same as heading → delta=0 → vx=0, vy=0
            MPC_Control(2) = 0.0;               // v = 0
        }
        MPC_Control(3) = checkMotionMode();
        in_bridge = false;
        publishSentryOptimalSpeed(MPC_Control);

        return;
    }

    double target_distance = (localplanner->target_point - robot_cur_position).norm();
    // (Fix 22) Removed pre-emptive replan at 1.0m — it restarted motion near goal
    // causing oscillation. Replan is still triggered by checkReplanFlag() if needed.
    if(target_distance < 0.3 && (localplanner->motion_mode != 8)){
        // (Fix 22) Increased from 0.1m to 0.3m — old threshold was far below stopping
        // distance (v²/2a = 2.0²/7.0 ≈ 0.57m), causing consistent overshoot.
        ROS_WARN("[MPC] arrival_goal! dist=%.2f", target_distance);
        arrival_goal = true;
    }

    localplanner->getFightTrackingTraj(robot_cur_position, cost_time, robot_cur_yaw);  // 可以倒车模式，更加灵活,团战模式全部适合倒车


    if(localplanner->reference_path.size() > 2){
        checkMotionNormal(line_speed_temp);
        int ret = localplanner->solveNMPC(sentry_state);
        std_msgs::Bool solver_status_msg;
        if (ret == 0) {
            ROS_WARN("[TrackingManager] solveNMPC failed, skipping this cycle");
            solver_status_msg.data = false;
            m_solver_status_pub.publish(solver_status_msg);
            return;
        } else {
        solver_status_msg.data = true;
        m_solver_status_pub.publish(solver_status_msg);
        localplanner->getNMPCPredictXU(predict_input); // v phi a w
        localplanner->getNMPCPredictionDeque(state_deque, input_deque);


        bool collision = localplanner->checkfeasible();
        if(collision){
            ROS_WARN("MPC is not safe");
        }
        vislization_util->visCandidateTrajectory(state_deque);
        vislization_util->visReferenceTrajectory(localplanner->ref_trajectory);
        vislization_util->visObsCenterPoints(localplanner->obs_points);
        vislization_util->publishMPCPredictedPath(state_deque);
        vislization_util->publishMPCReferencePath(localplanner->ref_trajectory);

        replan_check_flag.insert(replan_check_flag.begin(), collision);
        if(replan_check_flag.size() > 30){  // (Fix 30) was 15 (Fix 23); widened to 30 to reduce
                                            // oscillation near dynamic obstacles (0.3s window)
            replan_check_flag.pop_back();
        }
        }
    }
    for(int i = 0; i < (int)state_deque.size() / 2; i++){
        std::cout<<state_deque[i].z()<<std::endl;
    }
    checkReplanFlag();


    v_ctrl = predict_input(0);
    phi_ctrl = predict_input(1);
    acc_ctrl = predict_input(2);
    angular_ctrl = predict_input(3);

    if(planningMode == planningType::DISPENSE){  // 摆脱模式处理
        double reverse_time = (ros::Time::now() - dispense_time).toSec();
        if(reverse_time > 0.5){
            replan_now = true;
        }
        v_ctrl = -0.5 * v_ctrl;  // 卡住你就倒车拐弯
    }


    Eigen::Vector4d MPC_Control;
    if (use_omega_output_) {
        // omega mode (new robot): pack (ω, unused, v, mode)
        // hit_bridge will send: vx = v, vy = 0, z_angle = ω
        MPC_Control(0) = angular_ctrl;          // ω → angle_target field
        MPC_Control(1) = 0.0;                   // unused → angle_current field
        MPC_Control(2) = v_ctrl;                // v → line_speed field
    } else {
        // omni mode (old robot, default): pack (heading, chassis_yaw, v, mode)
        // hit_bridge decomposes v into chassis body-frame vx/vy.
        // CRITICAL: Must use CHASSIS yaw (robot_cur_yaw), NOT gimbal/LiDAR yaw.
        // The MPC solves heading relative to chassis yaw, so the decomposition
        // must also use chassis yaw.  Using gimbal yaw introduced the
        // gimbal-chassis yaw difference as an error, causing wrong vx/vy
        // directions and speed oscillation when the gimbal rotates.
        MPC_Control(0) = phi_ctrl + robot_add_yaw;
        MPC_Control(1) = robot_cur_yaw + robot_add_yaw;  // chassis yaw (was: robot_lidar_yaw)
        MPC_Control(2) = v_ctrl;
    }
    MPC_Control(3) = checkMotionMode();

    in_bridge = localplanner->checkBridge();
    publishSentryOptimalSpeed(MPC_Control);
    std::cout<<"[END] end local planning"<<std::endl;
}

void tracking_manager::rcvGazeboRealPosCallback(const gazebo_msgs::ModelStatesConstPtr &state)
{
    /* 实时位置 */
    int robot_namespace_id = 0;
    for(int i = 0;i < state->name.size();i++){
        if(state->name[i] == "mbot"){
            robot_namespace_id = i;
            break;
        }
    }

    ros::Time end_time = ros::Time::now();
    double time = (end_time - control_time).toSec();
    control_time = ros::Time::now(); // 当此控制周期的时间
    double cost_time = (control_time - localplanner->start_tracking_time).toSec();

    robot_cur_position(0) = state->pose[robot_namespace_id].position.x;
    robot_cur_position(1) = state->pose[robot_namespace_id].position.y;
    robot_cur_position(2) = 0.0;

    global_map->odom_position(0) = state->pose[robot_namespace_id].position.x;
    global_map->odom_position(1) = state->pose[robot_namespace_id].position.y;
    global_map->odom_position(2) = state->pose[robot_namespace_id].position.z;

    double robot_x = state->pose[robot_namespace_id].position.x;
    double robot_y = state->pose[robot_namespace_id].position.y;

    double x = state->pose[robot_namespace_id].orientation.x;
    double y = state->pose[robot_namespace_id].orientation.y;
    double z = state->pose[robot_namespace_id].orientation.z;
    double w = state->pose[robot_namespace_id].orientation.w;
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    robot_cur_yaw = atan2f(siny_cosp, cosy_cosp);

    // yaw pitch roll
    robot_cur_posture(2) = atan2f(siny_cosp, cosy_cosp);
    robot_cur_posture(1) = asin(2 * (w * y - x * z));
    robot_cur_posture(0) = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));

    global_map->odom_posture = robot_cur_posture;

    double v_ctrl = 0.0;
    double phi_ctrl = 0.0;
    double acc_ctrl = 0.0;
    double angular_ctrl = 0.0;
    Eigen::Vector4d predict_input;

    //  判断一下是不是需要加入抵达终点的判断来set速度为0

    double line_speed = sqrt(pow(abs(state->twist[robot_namespace_id].linear.x), 2) + pow(abs(state->twist[robot_namespace_id].linear.y), 2));
    Eigen::Vector2d robot_accvel;

    robot_cur_speed(0) = state->twist[robot_namespace_id].linear.x;
    robot_cur_speed(1) = state->twist[robot_namespace_id].linear.y;

    Eigen::VectorXd sentry_state;
    std::vector<Eigen::Vector4d> state_deque;
    std::vector<Eigen::Vector2d> input_deque;

    sentry_state = Eigen::VectorXd::Zero(4);
    sentry_state << robot_x, robot_y, line_speed, robot_cur_yaw;


    localplanner->robot_cur_pitch = asin(-2 * (x * z - w * y));

    if(localplanner->m_get_global_trajectory){
        arrival_goal = false;
    }
    if(arrival_goal){
        v_ctrl = 0.0;
        phi_ctrl = robot_cur_yaw;
        acc_ctrl = 0.0;
        angular_ctrl = 0.0;
        Eigen::Vector2d optimal_differential_speed;
        Eigen::Vector4d MPC_Control;

        optimal_differential_speed = getDifferentialModelSpeed(v_ctrl, angular_ctrl, robot_wheel_tread);
        publishMbotOptimalSpeed(optimal_differential_speed);

        MPC_Control(0) = v_ctrl;
        MPC_Control(1) = phi_ctrl;
        MPC_Control(2) = angular_ctrl;
        MPC_Control(3) = checkMotionMode();
//        MPC_Control(3) = 1;
        in_bridge = false;
        publishSentryOptimalSpeed(MPC_Control);
        return;
    }

    double target_distance = (localplanner->target_point - robot_cur_position).norm();
    if(target_distance < 0.1){
        ROS_WARN("[MPC] arrival_goal!");
        arrival_goal = true;
    }

    localplanner->getFightTrackingTraj(robot_cur_position, cost_time, robot_cur_yaw);  // 可以倒车模式，更加灵活


    if(localplanner->reference_path.size() > 2){
        checkMotionNormal(line_speed);

        int ret = localplanner->solveNMPC(sentry_state);
        std_msgs::Bool solver_status_msg;
        if (ret == 0) {
            ROS_WARN("[TrackingManager] solveNMPC failed, skipping this cycle");
            solver_status_msg.data = false;
            m_solver_status_pub.publish(solver_status_msg);
            return;
        } else {
        solver_status_msg.data = true;
        m_solver_status_pub.publish(solver_status_msg);
        localplanner->getNMPCPredictXU(predict_input); // v phi a w
        localplanner->getNMPCPredictionDeque(state_deque, input_deque);
        bool collision = localplanner->checkfeasible();
        if(collision){
            ROS_WARN("MPC is not safe");
        }
        vislization_util->visCandidateTrajectory(state_deque);
        vislization_util->visReferenceTrajectory(localplanner->ref_trajectory);
        vislization_util->visObsCenterPoints(localplanner->obs_points);
        vislization_util->publishMPCPredictedPath(state_deque);
        vislization_util->publishMPCReferencePath(localplanner->ref_trajectory);

        replan_check_flag.insert(replan_check_flag.begin(), collision);
        if(replan_check_flag.size() > 250){ // 检测num帧的数据防止碰撞，实际检测秒数为num/800帧
            replan_check_flag.pop_back();
        }
        }
    }
    checkReplanFlag();

    v_ctrl = predict_input(0);
    phi_ctrl = predict_input(1);
    acc_ctrl = predict_input(2);
    angular_ctrl = predict_input(3);

    /* 获取四轮差速模型的左右轮轮速 */
    Eigen::Vector2d optimal_differential_speed;
    Eigen::Vector4d MPC_Control;

    if(planningMode == planningType::DISPENSE){
        double reverse_time = (ros::Time::now() - dispense_time).toSec();
        if(reverse_time > 0.6){
            replan_now = true;
        }
        v_ctrl = -0.5 * v_ctrl;  // 卡住你就倒车拐弯
    }

    optimal_differential_speed = getDifferentialModelSpeed(v_ctrl, angular_ctrl, robot_wheel_tread);
    publishMbotOptimalSpeed(optimal_differential_speed);

    MPC_Control(0) = v_ctrl;
    MPC_Control(1) = phi_ctrl;
    MPC_Control(2) = angular_ctrl;
    MPC_Control(3) = checkMotionMode();
//    if(isxtl && localplanner->checkXtl()){
//        MPC_Control(3) = checkMotionMode();
//    }else{
//        MPC_Control(3) = 0;
//    }
    in_bridge = localplanner->checkBridge();
    publishSentryOptimalSpeed(MPC_Control);
}

/**
 * @brief 用线速度和角速度映射到左右轮轮速
 * @param
 */
Eigen::Vector2d tracking_manager::getDifferentialModelSpeed(double line_speed, double angular, double wheel_tread)
{
    Eigen::Vector2d LR_speed;
    LR_speed(0) = line_speed + wheel_tread / 2 * angular;
    LR_speed(1) = line_speed - wheel_tread / 2 * angular;
    return LR_speed;
}



int tracking_manager::checkMotionMode(){
    /**
     * 判断底盘运动模式,分别为正常运行，可倒车模式，慢速陀螺/小虎步，快速陀螺
     */
    int xtl_mode = 0;
    double time = accumulate(localplanner->duration_time.begin(), localplanner->duration_time.end(), 0.0);
    if(!isxtl){  // 正常运行，可倒车模式
        if(arrival_goal){
            xtl_mode = 2;
        }
        else if(time > 0.5 && !localplanner->checkBridge() && planningMode != planningType::DISPENSE){
           //  time > 2.0 &&
           xtl_mode = 0;  // 轨迹够长且不在桥洞里，在桥洞里不准转弯
        }else{
            xtl_mode = 1;
        }
    }else if(!arrival_goal && isxtl && localplanner->checkXtl()){
        if(is_attacked || localplanner->motion_mode == 8 || localplanner->motion_mode == 11){
            xtl_mode = 2;
        }else{
            xtl_mode = 0;
        }
//        xtl_mode = 2;
    }else if(arrival_goal && isxtl && localplanner->checkXtl()){
        if(is_attacked || localplanner->motion_mode == 11){
            xtl_mode = 3;
        }else{
            xtl_mode = 2;
        }
//        xtl_mode = 2;
    }else{
        if(localplanner->checkBridge()){
            xtl_mode = 1;
        }else{
            xtl_mode = 0;
        }
    }
    return xtl_mode;
}

void tracking_manager::checkReplanFlag(){
    // Fix 38b: Track when the last trajectory was received for periodic replan.
    static ros::Time last_traj_time = ros::Time::now();
    if(localplanner->m_get_global_trajectory){
        replan_check_flag.clear();
        localplanner->m_get_global_trajectory = false;
        last_traj_time = ros::Time::now();
    }
    int num_collision_error = accumulate(replan_check_flag.begin(), replan_check_flag.end(), 0);
    int num_tracking_low = accumulate(localplanner->tracking_low_check_flag.begin(), localplanner->tracking_low_check_flag.end(), 0);
    std_msgs::Bool replan_flag;

    // Fix 38b: Periodic replan every 5s to handle cleared obstacles.
    // When a dynamic obstacle moves away, the old detour path stays active
    // because checkfeasible() only triggers replan on collision (not on
    // "path is now suboptimal").  Periodic replan lets trajectory_generation
    // find a shorter path through the now-clear area.
    static const double PERIODIC_REPLAN_SEC = 5.0;
    bool periodic_replan = false;
    if (!arrival_goal && (ros::Time::now() - last_traj_time).toSec() > PERIODIC_REPLAN_SEC) {
        periodic_replan = true;
        last_traj_time = ros::Time::now();
        ROS_INFO("[Fix 38b] Periodic replan triggered (%.1fs since last trajectory)", PERIODIC_REPLAN_SEC);
    }

    if(num_collision_error > 25 || num_tracking_low > 4 || replan_now || periodic_replan){
        // Fix 42c: Break the perpetual replan cycle.
        // When MPC path goes through obstacles, checkfeasible() fires every
        // frame → num_collision_error hits 25 in ~0.6s → replan → new
        // trajectory → MPC restarts cold → same problem → replan again.
        // This positive feedback loop prevents the MPC from ever stabilizing.
        //
        // Fix: After receiving a new trajectory, give the MPC 3 seconds to
        // settle before allowing collision-triggered replanning.  Off-course
        // detection and periodic replan still work during cooldown.
        double time_since_traj = (ros::Time::now() - last_traj_time).toSec();
        bool collision_replan_ok = time_since_traj > 3.0;
        bool is_collision_only_trigger = (num_collision_error > 25)
                                       && !(num_tracking_low > 4)
                                       && !replan_now && !periodic_replan;
        if (is_collision_only_trigger && !collision_replan_ok) {
            // Skip collision-triggered replan during cooldown
            replan_flag.data = false;
            global_replan_pub.publish(replan_flag);
            return;
        }
        ROS_ERROR("need to replan now !!");
        planningMode = planningType::FASTMOTION;
        replan_flag.data = true;
    }
    else{
        replan_flag.data = false;
    }
    global_replan_pub.publish(replan_flag);
}

void tracking_manager::checkMotionNormal(double line_speed)
{
    /**
     * @brief 检查tracking是否正常，是否被卡住需要摆脱
     */
     if(localplanner->m_get_global_trajectory){
         planningMode = planningType::FASTMOTION;
         replan_now = false;
         start_checking_time = ros::Time::now();
         speed_check.clear();
     }
    speed_check.insert(speed_check.begin(), line_speed);
    double tracking_time = (ros::Time::now() - start_checking_time).toSec();
    double max_speed = 1.0;
    // Fix 35d: Tightened DISPENSE stuck-detection thresholds.
    // Old: tracking_time > 1.2, max_speed < 0.3, average_speed < 0.1
    // These triggered during MPC oscillation (sign-flipping speeds near zero)
    // and made DISPENSE reverse v_ctrl, causing a vicious feedback loop.
    // New: require 2.0s of truly stuck motion (max < 0.15, avg < 0.05).
    if(tracking_time > 2.0){
        speed_check.pop_back();
        max_speed = *max_element(speed_check.begin(), speed_check.end());
        double average_speed = std::accumulate(speed_check.begin(), speed_check.end(), 0.0) / speed_check.size();
        if(max_speed < 0.15 && !arrival_goal && average_speed < 0.05){
//            ROS_ERROR("dispense form now!");
            if(planningMode != planningType::DISPENSE){
                ROS_ERROR("get dispense time");
                dispense_time = ros::Time::now();  // 进入摆脱模式时进行计时
            }
            planningMode = planningType::DISPENSE;
        }

    }
}


void tracking_manager::publishMbotOptimalSpeed(Eigen::Vector2d &speed)
{
    std_msgs::Float64 left_speed;
    std_msgs::Float64 right_speed;
    sentry_msgs::slaver_speed sentry_speed;

    /* 抵达规划路径终点时立即停止移动 */
    left_speed.data = speed(0) / 0.1;
    right_speed.data = speed(1) / 0.1;

    tracking_manager::robot_left_speed_pub.publish(left_speed);
    tracking_manager::robot_right_speed_pub.publish(right_speed);
}

void tracking_manager::publishSentryOptimalSpeed(Eigen::Vector4d &speed)
{   // TODO 改为x y v w theta
    // v phi a w
    sentry_msgs::slaver_speed sentry_speed;
    std::pair<int, int> count_mode = {0,0};  /// max_id max_count

    motion_mode_vec.insert(motion_mode_vec.begin(), int(speed(3)));
    if(motion_mode_vec.size() > 40){
        motion_mode_vec.pop_back();
    }
    for(int i = 0; i < 4; i++){
        if(std::count(motion_mode_vec.begin(), motion_mode_vec.end(), i) > count_mode.second){
            count_mode.first = i;
            count_mode.second = std::count(motion_mode_vec.begin(), motion_mode_vec.end(), i);
        }
    }

//    std::cout<<"mpc_control(3): "<<speed(3)<<"count_mode.first: "<<count_mode.first<<std::endl;


    /* 抵达规划路径终点时立即停止移动 */

    sentry_speed.angle_target = speed(0);  // 目标yaw角度
    sentry_speed.angle_current = speed(1);  // 当前的雷达yaw角度
//    sentry_speed.angle_current = speed(1) + M_PI;  // 当前的雷达yaw角度
    sentry_speed.line_speed = speed(2);  // 目标线速度
    if(count_mode.second > 38){
        sentry_speed.xtl_flag = count_mode.first;  // 目标模式
    }else{
        sentry_speed.xtl_flag = last_motion_xtl_flag;  // 目标模式
    }
//    sentry_speed.xtl_flag = count_mode.first;  // 目标模式
    last_motion_xtl_flag = sentry_speed.xtl_flag;

//    sentry_speed.xtl_flag = 0;  // 目标模式
    sentry_speed.in_bridge = in_bridge;   // 是否进入桥洞

    std_msgs::Float64 robot_yaw;
    robot_yaw.data = tracking_manager::robot_cur_yaw;

    tracking_manager::sentry_speed_pub.publish(sentry_speed);
    robot_cur_yaw_pub.publish(robot_yaw);
}

