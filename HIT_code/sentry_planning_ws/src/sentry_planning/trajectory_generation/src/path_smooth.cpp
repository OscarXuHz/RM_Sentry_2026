#include "../include/path_smooth.h"
#include <iostream>


Smoother::~Smoother()
{}

void Smoother::setGlobalMap(std::shared_ptr<GlobalMap> &_global_map)
{
    global_map = _global_map;
}

void Smoother::init(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel, double desire_speed)
{
    pathSample(global_path, start_vel);
    if(path.size()<2){
        ROS_ERROR("[Smooth Init] path.size()<2!!  No path");
        return;
    }
    desire_veloity = desire_speed;
    headP = path[0];
    Eigen::Vector2d start_vels;
    start_vels.x() = start_vel.x();
    start_vels.y() = start_vel.y();

    getGuidePath(start_vels);
    if(!init_vel){  // 速度方向反向过大直接倒车不拐弯
        start_vels.x() = 0.0;
        start_vels.y() = 0.0;
    }

    tailP = path[path.size() - 1];
    pieceN = path.size() - 1;
    cubSpline.setConditions(headP, start_vels, tailP, pieceN);
}

bool Smoother::getObsPosition(double start_x, double start_y, double start_z,
                    double edge_x, double edge_y, double edge_z, Eigen::Vector3d &obs_pt)
{
    double path_len = std::sqrt((edge_x - start_x) * (edge_x - start_x) + (edge_y - start_y) * (edge_y - start_y));
    int n = static_cast<int>(path_len * global_map->m_inv_resolution * 2) + 1;
    double delta_x = (edge_x - start_x) / n;
    double delta_y = (edge_y - start_y) / n;
    int x_idx, y_idx, z_idx;

    for(int i = 0; i < n; i++){
        double temp_x = start_x + delta_x * i;
        double temp_y = start_y + delta_y * i;
        double temp_z = start_z;
        /* 用空间点坐标找到其所对应的栅格 */
        global_map->coord2gridIndex(temp_x, temp_y, temp_z, x_idx, y_idx, z_idx);
        if (!isFree(x_idx, y_idx, z_idx)) {
            obs_pt = {temp_x, temp_y, temp_z};
            return true;
        }
    }
    return false;

}

double Smoother::costFunction(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
    clock_t time_1 = clock();

    auto instance = reinterpret_cast<Smoother *>(ptr);
    const int points_num = instance->pieceN - 1;
    Eigen::Matrix2Xd grad;
    grad.resize(2, points_num);
    grad.setZero();
    double cost = 0.0;

    Eigen::Matrix2Xd inPs;
    inPs.resize(2, points_num);
    inPs.row(0) = x.head(points_num);
    inPs.row(1) = x.tail(points_num);
    Eigen::VectorXd inTimes;
    inTimes.resize(instance->pieceN);

    for(int i = 0; i < instance->pieceN; i++){
        inTimes(i) = 0.3 / instance->desire_veloity;  // 0.3对应分段采样间距0.3m
    }
    instance->cubSpline.setInnerPoints(inPs, inTimes);

    if(!instance->init_obs) {
        for (int i = 0; i < points_num; ++i) {
            double nearest_cost = 0.0;
            Eigen::Vector2d x_cur = inPs.col(i);
            Eigen::Vector2d obstacleGrad = instance->obstacleTerm(i, x_cur, nearest_cost);
        }
    }
    instance->init_obs = true;

    /// 整个优化过程最最最耗时的地方
    double energy;
    Eigen::Matrix2Xd energy_grad;
    Eigen::VectorXd energyT_grad, energyTau_grad;
    energy_grad.resize(2, points_num);
    energy_grad.setZero();

    energyT_grad.resize(instance->pieceN);
    energyT_grad.setZero();
    energyTau_grad.resize(instance->pieceN);
    energyTau_grad.setZero();

    instance->cubSpline.getEnergy(energy);  // 能量损失cost和梯度
    instance->cubSpline.getGradSmooth(energy_grad, energyT_grad);
    cost += energy;
    grad += energy_grad;

    Eigen::Matrix2Xd potential_grad;
    potential_grad.resize(2, points_num);
    potential_grad.setZero();
    for(int i = 0; i<points_num; ++i){
        double nearest_cost = 0.0;
        Eigen::Vector2d x_cur = inPs.col(i);
        Eigen::Vector2d obstacleGrad = instance->obstacleTerm(i, x_cur, nearest_cost);

        potential_grad.col(i) = obstacleGrad;
        cost += nearest_cost;
    }
    clock_t time_2 = clock();
    grad += potential_grad;

    g.setZero();
    // 控制点和时间梯度
    g.head(points_num) = grad.row(0).transpose();
    g.tail(points_num) = grad.row(1).transpose();
    clock_t time_3 = clock();
    return cost;
}

void Smoother::pathSample(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel)
{
    std::vector<Eigen::Vector2d> trajectory_point;
    double step = 0.3;
    double last_dis = 0.0;  /// 上一段留下来的路径
    path.clear();
    trajectory_point.clear();
    if(global_path.size()<2){
        ROS_ERROR("[Smooth Sample] global_path.size()<2, No path");
        return;
    }

//    if(start_vel.norm() > 1.0){  // 小trick
//        step = 0.3;
//    }

    for (int i = 0;i<global_path.size() - 1;i++)
    {
        Eigen::Vector2d start(global_path[i].x(), global_path[i].y());
        Eigen::Vector2d end(global_path[i+1].x(), global_path[i+1].y());
        Eigen::Vector2d start2end(global_path[i+1].x() - global_path[i].x(), global_path[i+1].y() - global_path[i].y());
        double path_distance = std::sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
        /// 每一段的距离
        if(trajectory_point.empty()){
            trajectory_point.push_back(start);
        }
        /// 将上一段路径残留的部分放进去
        Eigen::Vector2d start_new;
        if(path_distance>=(step - last_dis)){
            start_new.x() = start.x() + start2end.x() * (step - last_dis)/path_distance;
            start_new.y() = start.y() + start2end.y() * (step - last_dis)/path_distance;
//            trajectory_point.push_back(start_new);
            step = 0.3;  // 无论速度多大我们只调整第一段的长度，push_back后直接回归低采样
        }
        else  /// 这段路径太短了，加上last_dis还没有采样步长长，因此直接加到dis_last中
        {
            last_dis = last_dis + path_distance;  /// TODO 这种处理方式有个问题，容易把角点扔掉，不保证安全性
            continue;
        }

        Eigen::Vector2d new_start2end(end.x() - start_new.x(), end.y() - start_new.y());
        double path_distance_new = std::sqrt(pow(start_new.x() - end.x(), 2) + pow(start_new.y() - end.y(), 2));
        int sample_num = (int)((path_distance_new)/step);
        if(path_distance_new < 0.1){
            last_dis = (path_distance-(step-last_dis)) - ((float)sample_num) * step;
            trajectory_point.push_back(start_new);
            continue;
        }

        for (int j = 0;j<=sample_num; j++){
            Eigen::Vector2d sample_point(start_new.x() + new_start2end.x() * step/path_distance_new * j,
                            start_new.y() + new_start2end.y() * step/path_distance_new * j);
            trajectory_point.push_back(sample_point);
        }
        last_dis = (path_distance-(step-last_dis)) - ((float)sample_num) * step;
    }

    if(last_dis < step/2.0f && trajectory_point.size()>=2)
    {
        trajectory_point.erase((trajectory_point.end()-1));
    }
    Eigen::Vector2d end(global_path[global_path.size() - 1].x(),global_path[global_path.size() - 1].y());
    trajectory_point.push_back(end);
    path.assign(trajectory_point.begin(), trajectory_point.end());
}

void Smoother::getGuidePath(Eigen::Vector2d start_vel, double radius){
    if(path.size()<3){
        ROS_ERROR("[Smooth Path] path.size()<3, No need to optimize");
        return;
    }
    Eigen::Vector2d direction = path[1] - path[0];
    Eigen::Vector2d start_point = path[0];
    direction = direction / direction.norm();

    double theta = acos((start_vel.x() * direction.x() + start_vel.y() * direction.y()) / (direction.norm() * start_vel.norm()));

    if(theta < M_PI_2 * 4/3 && start_vel.norm() > 0.2){
        init_vel = true;
    }else{
        init_vel = false;
    }
}

void Smoother::smoothPath()
{
    allobs.clear();
    mid_distance.clear();
    init_obs = false;
    init_vel = false;
    // TODO 时间维度N 空间维度2(N-1)
    if(path.size()<3){
        ROS_ERROR("[Smooth Path] path.size()<3, No need to optimize");
        return;
    }
    Eigen::VectorXd x(pieceN * 2 - 2);

    for(int i = 0; i < pieceN - 1; i++){  // 控制点
        x(i) = path[i + 1].x();
        x(i + pieceN - 1) = path[i + 1].y();
    }

    obs_coord.clear();

    double minCost = 0.0;
    lbfgs_params.mem_size = 64;  // 32
    lbfgs_params.past = 5;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 2.0e-5;
    lbfgs_params.delta = 2e-5;  // 3e-4
    lbfgs_params.max_linesearch = 32;  // 32
    lbfgs_params.f_dec_coeff = 1.0e-4;
    lbfgs_params.s_curv_coeff = 0.9;
    int ret = lbfgs::lbfgs_optimize(x,
                                    minCost,
                                    &Smoother::costFunction,
                                    nullptr,
                                    this,
                                    lbfgs_params);

    if (ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
    {
        if(ret > 0){
            ROS_DEBUG_STREAM("[Smooth Optimize] Optimization Success: "
                             << lbfgs::lbfgs_stderr(ret));
        }else if (ret == 0){
            ROS_INFO_STREAM("[Smooth Optimize] Optimization STOP: "
                                     << lbfgs::lbfgs_stderr(ret));
        }
        else{
            ROS_INFO_STREAM("[Smooth Optimize] Optimization reaches the maximum number of evaluations: "
                                    << lbfgs::lbfgs_stderr(ret));
        }

        for(int i = 0; i < pieceN - 1; i++){
            path[i + 1].x() = x(i);
            path[i + 1].y() = x(i + pieceN - 1);
        }
        pathInPs.resize(2, pieceN - 1);
        pathInPs.row(0) = x.head(pieceN - 1);
        pathInPs.row(1) = x.segment(pieceN - 1, pieceN - 1);
    }
    else
    {
        path.clear();
        ROS_ERROR_STREAM("[Smooth Optimize] Optimization Failed: "
                                << lbfgs::lbfgs_stderr(ret));

    }
}

void Smoother::pathResample()
{
    ROS_INFO("[Smooth Resample] resample");
    finalpath.clear();
    std::vector<Eigen::Vector2d> trajectory_point;
    const int step = 2;
    trajectory_point.clear();
    if(path.size()<2){
        ROS_ERROR("[Smooth Resample] path Resample.size()<3, No path");
        return;
    }
    trajectory_point.push_back(path[0]);

    int sample_num = int((path.size() - 1)/step);
    for (int i = 1; i<sample_num; i++){
        trajectory_point.push_back(path[i*step]);
    }


    if(int(path.size() -1) > step*sample_num){
        trajectory_point.push_back((path.back()));
    }
    finalpath.assign(path.begin(), path.end());
}

std::vector<Eigen::Vector2d> Smoother::getPath() /// 优化过后的最终轨迹控制点
{
    return finalpath;
}

std::vector<Eigen::Vector2d> Smoother::getSamplePath(){  /// 优化前的采样点(可视化用)
    return path;
}

Eigen::Vector2d Smoother::obstacleTerm(int idx, Eigen::Vector2d xcur, double &nearest_cost)
{
    nearest_cost = 0.0;
    Eigen::Vector2d gradient(0.0, 0.0);
    double R = 0.3;  /// 防碰撞半径
    if(!init_obs)
    {
        obs_coord.clear();
        getObsEdge(xcur);
        allobs.push_back(obs_coord);

        std::vector<double> distance_temp;
        double ddd = 0.0;
        for(int i = 0; i < obs_coord.size(); i++)
        {
            double distance = std::sqrt((xcur(0) - obs_coord[i](0)) * (xcur(0) - obs_coord[i](0)) +
                                        (xcur(1) - obs_coord[i](1)) * (xcur(1) - obs_coord[i](1)));
            if(distance > (R * 1.2)){
                continue;
            }
            distance_temp.push_back(distance);
            ddd += distance;
        }

        if(!distance_temp.empty()){
            mid_distance.push_back(ddd/distance_temp.size());   /// 均值阈值用于处理狭窄路段
        }else{
            mid_distance.push_back(R);
        }

        return gradient;
    }

    int size = allobs[idx].size();
    for(int i = 0; i < allobs[idx].size(); i++)
    {

        double tempR = mid_distance[idx];
        double distance = std::sqrt((xcur(0) - allobs[idx][i](0)) * (xcur(0) - allobs[idx][i](0)) + (xcur(1) - allobs[idx][i](1)) * (xcur(1) - allobs[idx][i](1)));

        if(distance > tempR || distance < 1e-10){
            continue;
        }

        nearest_cost += wObstacle * (tempR - distance) * (tempR - distance)  / (double)size;
        Eigen::Vector2d obsVct(xcur.x() - allobs[idx][i](0), xcur.y() - allobs[idx][i](1));
        gradient += wObstacle * 2 * (obsVct / distance) * (distance - tempR) / (double)size;
    }
    return gradient;
}


void Smoother::getObsEdge(Eigen::Vector2d xcur)
{
    Eigen::Vector3d start_pt;
    start_pt(0) = xcur(0);
    start_pt(1) = xcur(1);
    start_pt(2) = 0.0;
    Eigen::Vector3i start_idx = global_map->coord2gridIndex(start_pt);
    start_pt = global_map->gridIndex2coord(start_idx);

    double start_x = start_pt(0);
    double start_y = start_pt(1);
    double start_z = 0.0;
    double R = 4.0;

    for(int i = -8; i <= 8; i++) /// 这里搜索范围太小会导致一系列的问题，包括但不限于小收敛误差需求时导致的撞墙，
        /// 因为你搜不到较远处的障碍物，但是太多了就会严重拖慢速度。
    {
        for (int j = -8; j <=8; j ++)
        {
            int x_idx = std::max(start_idx.x() + i, 0);
            int y_idx = std::max(start_idx.y() + j, 0);
            int z_idx = start_idx.z();

            Eigen::Vector3i temp_idx = {x_idx, y_idx, z_idx};

            if (!isFree(x_idx, y_idx, z_idx)) {
                Eigen::Vector3d obs_pt = global_map->gridIndex2coord(temp_idx);
                obs_coord.push_back(obs_pt);
            }
        }
    }

    for(int i = 0; i < 100; i++){   /// 搜索最近2.5m范围内的点
        double edge_x = start_x + sin(i * M_PI / 50) * R;
        double edge_y = start_y + cos(i * M_PI / 50) * R;
        double edge_z = start_z;

        Eigen::Vector3d obs_pt;
        if(getObsPosition(start_x, start_y, start_z, edge_x, edge_y, edge_z, obs_pt))
        {
            if((obs_pt - start_pt).norm() > 1.0){
                obs_coord.push_back(obs_pt);
            }
        }
    }
}


bool Smoother::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return ((global_map->data[idx_x * global_map->GLY_SIZE + idx_y] < 1 && global_map->l_data[idx_x * global_map->GLY_SIZE + idx_y] < 1));
}

bool Smoother::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

void Smoother::forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T)
{
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++)
    {
        T(i) = tau(i) > 0.0
               ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
               : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
    return;
}

void Smoother::backwardT(const Eigen::VectorXd &T, Eigen::VectorXd &tau)
{
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++)
    {
        tau(i) = T(i) > 1.0
                 ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                 : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }

    return;
}

void Smoother::backwardGradT(const Eigen::VectorXd &tau, Eigen::VectorXd &gradT, Eigen::VectorXd &gradTau)
{
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;
    for (int i = 0; i < sizeTau; i++){
        if (tau(i) > 0)
        {
            gradTau(i) = gradT(i) * (tau(i) + 1.0);
        }
        else
        {
            denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
            gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
        }
    }
    return;
}
