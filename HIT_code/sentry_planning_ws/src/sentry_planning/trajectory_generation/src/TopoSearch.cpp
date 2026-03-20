//
// Created by zzt on 23-10-22.
//

#include "../include/TopoSearch.h"

void TopoSearcher::init(ros::NodeHandle &nh, std::shared_ptr<GlobalMap> &_global_map)
{
    m_graph.clear();
    m_eng = std::default_random_engine(m_rd());
    m_rand_pos = std::uniform_real_distribution<double>(0.0, 1.0);
    max_sample_num = 1000;

    m_sample_inflate(0) = 2.0;
    m_sample_inflate(1) = 7.0;

    global_map = _global_map;
}

void TopoSearcher::createLocalGraph(Eigen::Vector3d start, Eigen::Vector3d end, bool attack_target){

    // 局部重规划，局部重搜索
    ros::Time t1, t2;  /// 计算时间
    t1 = ros::Time::now();
    for(int i = 0; i<m_graph.size(); i++){  /// 注意shared_ptr引用计数问题
        m_graph[i]->neighbors.clear();
        m_graph[i]->neighbors_but_noconnected.clear();
    }
    m_graph.clear();

    // Override z with BEV map height for consistency with topo graph heights
    {
        Eigen::Vector3i si = global_map->coord2gridIndex(start);
        start.z() = global_map->getHeight(si.x(), si.y());
        Eigen::Vector3i ei = global_map->coord2gridIndex(end);
        end.z() = global_map->getHeight(ei.x(), ei.y());
    }

    GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
    GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));
    m_graph.push_back(start_node);
    m_graph.push_back(end_node);

    m_sample_r(0) = 0.5 * (end - start).norm() + m_sample_inflate(0);
    m_sample_r(1) = m_sample_inflate(1);

    m_translation = 0.5 * (start + end);  // 采样中心
    Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
    xtf = (end - m_translation).normalized();
    ytf = xtf.cross(downward).normalized();
    ztf = xtf.cross(ytf);

    m_rotation.col(0) = xtf;
    m_rotation.col(1) = ytf;
    m_rotation.col(2) = ztf;

    m_rotation_inv = m_rotation.transpose();

    int node_id = 1;

    for(int i = 0; i < global_map->topo_keypoint.size(); i++)  // 只对关键点(路口点)进行采样
    {
        double min_distance = 0.0;
        int exist_unvisiual = 0;
        Eigen::Vector3d pt = global_map->topo_keypoint[i];
        std::vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt, min_distance, exist_unvisiual);

        // Always add keypoints near start/end to ensure graph connectivity
        double dist_to_start = (pt - start).head<2>().norm();
        double dist_to_end   = (pt - end).head<2>().norm();
        bool near_terminal = (dist_to_start < 5.0 || dist_to_end < 5.0);

        if(visib_guards.size() < 1) { // 这个采样点是不可见的，所以直接建立守卫点
            GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
            m_graph.push_back(guard);

        }else {
            if(min_distance > 2.0 || near_terminal){
                GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
                m_graph.push_back(guard);
            }
        }
    }

    int sample_num = 0;
    Eigen::Vector3d pt;

    while(sample_num < (max_sample_num/2)){
        double min_distance = 0.0;
        int exist_unvisiual = 0;
        pt = getLocalSample();  /// 返回的采样点
        Eigen::Vector3i pt_idx = global_map->coord2gridIndex(pt);
        ++sample_num;
        if(global_map->isOccupied(pt_idx, false)) {
            continue;
        }

        std::vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt, min_distance, exist_unvisiual);   /// 根据采样点找可见的守卫点

        if(visib_guards.size() < 1) { // 这个采样点是不可见的，所以直接建立守卫点
            GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
            m_graph.push_back(guard);
        }
        else if (visib_guards.size() > 1){  // 这个采样点可以被两个守卫点看到，建立连接点
            checkHeightFeasible(visib_guards[0], visib_guards[1], pt, node_id);
        }
        else if(visib_guards.size() == 1){  // 当只能被一个守卫点看到时，需要判断是否可以建立连接点
            if(exist_unvisiual > 1){  // 这个的意义在于如果迭代到最后都只有一个点可见的话这个点就是一个孤点，没有意义直接扔
                continue;
            }
            int obs_num = 0;
            double start_x = pt.x();
            double start_y = pt.y();
            double start_z = pt.z();

            for (int i = 0; i<40; i++){  // 找0.5m内的占用栅格数量
                double edge_x = start_x + sin(i * M_PI_4 /5) * 0.5;
                double edge_y = start_y + cos(i * M_PI_4/ 5) * 0.5;
                double edge_z = start_z;

                int pt_idx, pt_idy, pt_idz;
                global_map->coord2gridIndex(edge_x, edge_y, edge_z, pt_idx, pt_idy, pt_idz);
                if (global_map->isOccupied(pt_idx, pt_idy, pt_idz, false)){
                    obs_num ++;
                }
            }
            if(obs_num>=20){  // 如果周围的障碍物较多则直接建立守卫点，解决困难样本
                GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
                m_graph.push_back(guard);
                if(abs(pt.z() - visib_guards[0]->pos.z()) > 0.4){
                    continue;   // 高度差别太大，不建立守卫点
                }

            }
        }
    }

    // Ensure start/end nodes are connected (same fallback as createGraph)
    for(int ti = 0; ti <= 1; ti++) {
        if(m_graph[ti]->neighbors.empty()) {
            double best_dist = 1e9;
            int best_g = -1;
            for(int g = 0; g < (int)m_graph.size(); g++) {
                if(g == ti) continue;
                if(m_graph[g]->type_ != GraphNode::Guard) continue;
                double d = (m_graph[g]->pos - m_graph[ti]->pos).head<2>().norm();
                if(d < best_dist) {
                    Eigen::Vector3d pc;
                    if(lineVisib(m_graph[ti]->pos, m_graph[g]->pos, 0.2, pc, 0)) {
                        best_dist = d;
                        best_g = g;
                    }
                }
            }
            if(best_g >= 0) {
                m_graph[ti]->neighbors.push_back(m_graph[best_g]);
                m_graph[best_g]->neighbors.push_back(m_graph[ti]);
                ROS_WARN("[Topo Local] Force-connected %s node to guard %d (dist=%.2fm)",
                         ti == 0 ? "start" : "end", m_graph[best_g]->m_id, best_dist);
            }
        }
    }

    if(!attack_target){  // 正常模式下会直接出路径，追击模式下只用于生成局部追击轨迹
        ROS_WARN("[Topo Local]: complete sample");
        std::vector<std::vector<Eigen::Vector3d>> temp;  // 搜索可行路径并进行排序，找到最佳路径
        temp = searchPaths();

        // Same connectivity fallback as createGraph
        if(min_path.empty()) {
            std::vector<bool> reachable(m_graph.size(), false);
            std::queue<int> bfs_q;
            reachable[0] = true;
            bfs_q.push(0);
            while(!bfs_q.empty()) {
                int cur = bfs_q.front();
                bfs_q.pop();
                for(auto& nb : m_graph[cur]->neighbors) {
                    if(!reachable[nb->m_id]) {
                        reachable[nb->m_id] = true;
                        bfs_q.push(nb->m_id);
                    }
                }
            }
            if(!reachable[1]) {
                double best_dist = 1e9;
                int best_g = -1;
                for(int g = 0; g < (int)m_graph.size(); g++) {
                    if(g == 1) continue;
                    if(!reachable[g]) continue;
                    double d = (m_graph[g]->pos - m_graph[1]->pos).head<2>().norm();
                    if(d < best_dist) {
                        Eigen::Vector3d pc;
                        if(lineVisib(m_graph[1]->pos, m_graph[g]->pos, 0.2, pc, 0)) {
                            best_dist = d;
                            best_g = g;
                        }
                    }
                }
                if(best_g >= 0) {
                    m_graph[1]->neighbors.push_back(m_graph[best_g]);
                    m_graph[best_g]->neighbors.push_back(m_graph[1]);
                    ROS_WARN("[Topo Local] Force-connected end to reachable node %d (dist=%.2fm)",
                             m_graph[best_g]->m_id, best_dist);
                    temp = searchPaths();  // retry
                }
            }
        }

        ROS_DEBUG("[Topo Local]: sample time: %f", (ros::Time::now() - t1).toSec());
    }
}

bool TopoSearcher::searchOnePointPath(Eigen::Vector3d end){
    int node_id = m_graph.size() - 1;  // 节点id为当前图的大小
    // 找最近的点
    int exist_unvisiual = 0;
    std::vector<GraphNode::Ptr> visib_guards = findMinVisibGuard(end, exist_unvisiual);   /// 根据采样点找可见的守卫点

    if(visib_guards.size() < 2) { // 这个采样点是不可见的，所以我们认为他啥也没看到
        return false;
    }else if (visib_guards.size() == 2){  // 这个采样点可以被两个守卫点看到，建立连接点
        checkHeightFeasible(visib_guards[0], visib_guards[1], end, node_id);
    }

    std::vector<std::vector<Eigen::Vector3d>> temp;  // 搜索可行路径并进行排序，找到最佳路径
    temp = searchPaths(node_id);
    return true;
}

void TopoSearcher::createGraph(Eigen::Vector3d start, Eigen::Vector3d end)
{
    ros::Time t1, t2;  /// 计算时间
    t1 = ros::Time::now();
//    std::vector<GraphNode::Ptr> graph_temp;
//    m_graph.swap(graph_temp);
    for(int i = 0; i<m_graph.size(); i++)
    {  /// 注意shared_ptr引用计数问题
        m_graph[i]->neighbors.clear();
        m_graph[i]->neighbors_but_noconnected.clear();
    }
    m_graph.clear();

    // Override z with BEV map height so start/end are height-consistent
    // with the rest of the topo graph (which uses BEV heights exclusively).
    // SLAM odometry z can differ significantly from BEV heights.
    {
        Eigen::Vector3i si = global_map->coord2gridIndex(start);
        start.z() = global_map->getHeight(si.x(), si.y());
        Eigen::Vector3i ei = global_map->coord2gridIndex(end);
        end.z() = global_map->getHeight(ei.x(), ei.y());
    }

    GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
    GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));
    m_graph.push_back(start_node);
    m_graph.push_back(end_node);
    int node_id = 1;

    for(int i = 0; i < global_map->topo_keypoint.size(); i++)  // 只对关键点(路口点)进行采样
    {
        double min_distance = 0.0;
        int exist_unvisiual = 0;
        Eigen::Vector3d pt = global_map->topo_keypoint[i];
        std::vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt, min_distance, exist_unvisiual);

        // Always add keypoints near start/end to ensure graph connectivity
        double dist_to_start = (pt - start).head<2>().norm();
        double dist_to_end   = (pt - end).head<2>().norm();
        bool near_terminal = (dist_to_start < 5.0 || dist_to_end < 5.0);

        if(visib_guards.size() < 1) { // 这个采样点是不可见的，所以直接建立守卫点
            GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
            m_graph.push_back(guard);

        }else {
            if(min_distance > 2.0 || near_terminal){
                GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
                m_graph.push_back(guard);
            }
        }
    }
    int sample_num = 0;
    Eigen::Vector3d pt;
    ROS_DEBUG("[Topo]: m_graph before num: %d", m_graph.size());

    while(sample_num < max_sample_num)
    {  // 开始迭代
        double min_distance = 0.0;
        int exist_unvisiual = 0;
        pt = getSample();  /// 返回的采样点

        Eigen::Vector3i pt_idx = global_map->coord2gridIndex(pt);

        ++sample_num;
        std::vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt, min_distance, exist_unvisiual);   /// 根据采样点找可见的守卫点

        if(visib_guards.size() < 1) { // 这个采样点是不可见的，所以直接建立守卫点
            GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
            m_graph.push_back(guard);
        }
        else if (visib_guards.size() == 2){  // 这个采样点可以被两个守卫点看到，建立连接点
            checkHeightFeasible(visib_guards[0], visib_guards[1], pt, node_id);
        }
        else if(visib_guards.size() == 1){  // 当只能被一个守卫点看到时，需要判断是否可以建立连接点
            if(exist_unvisiual > 2){  // 这个的意义在于如果迭代到最后有很多点可见的话这个点就不是困难点，没有意义直接扔,阈值太小的话会多很多没必要的警戒点导致topo路径很奇怪
                continue;
            }
            if(min_distance < 0.5){
                continue;
            }
            int obs_num = 0;
            double start_x = pt.x();
            double start_y = pt.y();
            double start_z = pt.z();

            for (int i = 0; i<40; i++){  // 找0.5m内的占用栅格数量
                double edge_x = start_x + sin(i * M_PI_4 /5) * 0.5;
                double edge_y = start_y + cos(i * M_PI_4/ 5) * 0.5;
                double edge_z = start_z;

                int pt_idx, pt_idy, pt_idz;
                global_map->coord2gridIndex(edge_x, edge_y, edge_z, pt_idx, pt_idy, pt_idz);
                if (global_map->isOccupied(pt_idx, pt_idy, pt_idz, false)){
                    obs_num ++;
                }
            }

            if(obs_num>=10){  // 如果周围的障碍物较多则直接建立守卫点，解决困难样本

                int idx, idy, idz;
                global_map->coord2gridIndex(pt.x(), pt.y(), pt.z(), idx, idy, idz);
                // Fix 34a: Removed hardcoded pt.z()=0.08 for bridge areas.
                // second_height is never computed, so this just created phantom
                // guard points at the wrong z-height.
                GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
                m_graph.push_back(guard);

            }
        }
    }
    t2 = ros::Time::now();

    // Ensure start/end nodes are connected to the graph.
    // When start/end are far from topo keypoints (>5m) or in open areas,
    // the sampling process may fail to connect them.
    for(int ti = 0; ti <= 1; ti++) {
        if(m_graph[ti]->neighbors.empty()) {
            double best_dist = 1e9;
            int best_g = -1;
            for(int g = 0; g < (int)m_graph.size(); g++) {
                if(g == ti) continue;
                if(m_graph[g]->type_ != GraphNode::Guard) continue;
                double d = (m_graph[g]->pos - m_graph[ti]->pos).head<2>().norm();
                if(d < best_dist) {
                    Eigen::Vector3d pc;
                    if(lineVisib(m_graph[ti]->pos, m_graph[g]->pos, 0.2, pc, 0)) {
                        best_dist = d;
                        best_g = g;
                    }
                }
            }
            if(best_g >= 0) {
                m_graph[ti]->neighbors.push_back(m_graph[best_g]);
                m_graph[best_g]->neighbors.push_back(m_graph[ti]);
                ROS_WARN("[Topo] Force-connected %s node to guard %d (dist=%.2fm)",
                         ti == 0 ? "start" : "end", m_graph[best_g]->m_id, best_dist);
            }
        }
    }

    std::vector<std::vector<Eigen::Vector3d>> temp;  // 搜索可行路径并进行排序，找到最佳路径
    temp = searchPaths();

    // If Dijkstra failed, the end node may have neighbors but be in a
    // disconnected component from start.  BFS from start to find all
    // reachable nodes, then force-connect end to the nearest reachable
    // visible guard and retry.
    if(min_path.empty()) {
        std::vector<bool> reachable(m_graph.size(), false);
        std::queue<int> bfs_q;
        reachable[0] = true;
        bfs_q.push(0);
        while(!bfs_q.empty()) {
            int cur = bfs_q.front();
            bfs_q.pop();
            for(auto& nb : m_graph[cur]->neighbors) {
                if(!reachable[nb->m_id]) {
                    reachable[nb->m_id] = true;
                    bfs_q.push(nb->m_id);
                }
            }
        }
        if(!reachable[1]) {
            double best_dist = 1e9;
            int best_g = -1;
            for(int g = 0; g < (int)m_graph.size(); g++) {
                if(g == 1) continue;
                if(!reachable[g]) continue;
                double d = (m_graph[g]->pos - m_graph[1]->pos).head<2>().norm();
                if(d < best_dist) {
                    Eigen::Vector3d pc;
                    if(lineVisib(m_graph[1]->pos, m_graph[g]->pos, 0.2, pc, 0)) {
                        best_dist = d;
                        best_g = g;
                    }
                }
            }
            if(best_g >= 0) {
                m_graph[1]->neighbors.push_back(m_graph[best_g]);
                m_graph[best_g]->neighbors.push_back(m_graph[1]);
                ROS_WARN("[Topo] Force-connected end to reachable node %d (dist=%.2fm)",
                         m_graph[best_g]->m_id, best_dist);
                temp = searchPaths();  // retry
            } else {
                ROS_ERROR("[Topo] End node is unreachable and no visible reachable guard found");
            }
        }
    }

    ROS_DEBUG("[Topo]: dijkstra time: %f", (ros::Time::now() - t2).toSec());
    ROS_DEBUG("[Topo]: topo search time: %f", (ros::Time::now() - t1).toSec());
    ROS_DEBUG("[Topo]: m_graph num: %d", m_graph.size());
}

std::vector<std::vector<Eigen::Vector3d>> TopoSearcher::searchPaths(int node_id)
{
    raw_paths.clear();
//    std::vector<GraphNode::Ptr> visited;
//    visited.push_back(m_graph[0]);
//    depthFirstSearch(visited);  // 进行深度优先搜索得到部分可行路径
    DijkstraSearch(node_id);  // 改为dijkstra  默认为1

    return raw_paths;
}

void TopoSearcher::DijkstraSearch(int node_id){
    min_path.clear();
    std::vector<double> minDist(m_graph.size() + 1, 100000.0);

    std::vector<bool> visited(m_graph.size() + 1, false);
    Eigen::Vector3d temp = {0,0,0};
    std::vector<std::pair<int, Eigen::Vector3d>> parent (m_graph.size() + 1, std::make_pair(-1, temp));

    minDist[0] = 0;
    for (int i = 0; i < m_graph.size(); i++) { // 遍历所有节点
        int minVal = 100000.0;
        int cur_id = -1;
        int index = -1;
        Eigen::Vector3d temp_point;

        // 找距离源点最近(minDist最小)的id点
        for(int j = 0; j<m_graph.size(); j++){
            if(!visited[j] && minDist[j] < minVal){
                minVal = minDist[j];
                cur_id = j;
                temp_point = m_graph[j]->pos;
            }
        }

        if(i == 0 && cur_id < 0){
            if(m_graph[0]->neighbors.size() > 0){
                cur_id = 0;
                index = 0;
                temp_point = m_graph[i]->pos;
            }else{
                ROS_ERROR("[Topo Search] No Path, No point connect to start point");
                return;
            }
        }

        if(cur_id < 0){
            continue;
        }
;
        visited[cur_id] = true;
        for(int j = 0; j<m_graph[cur_id]->neighbors.size(); j++){
            int id_temp = m_graph[cur_id]->neighbors[j]->m_id;
            if(!visited[id_temp] && minDist[cur_id] + (m_graph[cur_id]->neighbors[j]->pos - temp_point).norm() < minDist[id_temp])
            {
                minDist[id_temp] = minDist[cur_id] + (m_graph[cur_id]->neighbors[j]->pos - temp_point).norm();
                parent[id_temp].first = cur_id;
                parent[id_temp].second = temp_point;
            }
        }
    }

    int index = node_id;  // TODO 这里根据终点的index进行更改变化
    if(minDist[index] > 10000.0){
        if(node_id == 1){
            ROS_ERROR("[Topo Search] No Path, No point connect to end point");
        }
        return;
    }
    min_path.push_back(m_graph[index]->pos);
    while(1){
        min_path.push_back(parent[index].second);
        index = parent[index].first;
        if(index == 0){
            break;
        }
    }
//    std::cout<<"min_path size: "<<min_path.size()<<std::endl;
    std::reverse(min_path.begin(), min_path.end());

}

void TopoSearcher::depthFirstSearch(std::vector<GraphNode::Ptr>& vis)
{
    GraphNode::Ptr cur = vis.back();
    for(int i = 0; i < cur->neighbors.size(); ++i){

        if(vis.size() > 12)   /// 回溯路径禁止过长 仿真环境设置为18
        {
            return;
        }
        if(cur->neighbors[i]->m_id == 1){  /// 到达终点,递归产生的结果意味着一条闭合的路径已经产生了
            std::vector<Eigen::Vector3d> path;
            for (int j = 0; j < vis.size(); ++j){
                path.push_back(vis[j]->pos);
            }
            path.push_back(cur->neighbors[i]->pos);
            raw_paths.push_back(path);
            if (raw_paths.size() >= 800) return;  // 可能导致飞坡次数少，但是正好 仿真环境设置为600
            break;
        }
    }

    for (int i = 0; i < cur->neighbors.size(); ++i) {
        // skip reach goal
        if (cur->neighbors[i]->m_id == 1) continue;

        bool revisit = false;
        for (int j = 0; j < vis.size(); ++j) {
            if (cur->neighbors[i]->m_id == vis[j]->m_id) {  /// 表明当前的neighbors在当前循环中已经被visit了，禁止重复访问
                revisit = true;
                break;
            }
        }
        if (revisit) continue;

        // recursive search
        vis.push_back(cur->neighbors[i]);
        depthFirstSearch(vis);
        if (raw_paths.size() >= 800) return;

        vis.pop_back();
    }
}

void TopoSearcher::checkHeightFeasible(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt, int &node_id)
{   // 检查两个守卫点之间是否可以建立连接点,进行双向与单向链接
    int direction;

    GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));  //连接点

    if(heightFeasible(g1->pos, pt, direction)){  // 检查：g1守卫与pt是否高度可通行，可以则建立连接，否则建立单向连接或者不连接，g2同理
        g1->neighbors.push_back(connector);
        connector->neighbors.push_back(g1);
    }else{
        if(direction == 1){
            g1->neighbors.push_back(connector);
            connector->neighbors_but_noconnected.push_back(g1);

        }else if(direction == 2){
            g1->neighbors_but_noconnected.push_back(connector);
            connector->neighbors.push_back(g1);
        }
    }

    if(heightFeasible(g2->pos, pt, direction)){
        g2->neighbors.push_back(connector);
        connector->neighbors.push_back(g2);
    }else{
        if(direction == 1){
            g2->neighbors.push_back(connector);
            connector->neighbors_but_noconnected.push_back(g2);

        }else if(direction == 2){
            g2->neighbors_but_noconnected.push_back(connector);
            connector->neighbors.push_back(g2);
        }
    }
    m_graph.push_back(connector);
}

bool TopoSearcher::heightFeasible(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int &direction, double step, double thresh)
{
    /**
     * @brief 判断两点之间是否高度可通行
     */
    // p2是connector点, 如果双向可通行，返回true，否则返回false并计算连接方向
    double ray_ptx, ray_pty, ray_ptz;
    double p2_x, p2_y, p2_z, p1_x, p1_y, p1_z;
    double distance_thresh;
    p2_x = p2.x();
    p2_y = p2.y();
    p2_z = p2.z();

    p1_x = p1.x();
    p1_y = p1.y();
    p1_z = p1.z();

    double x_offset = p1.x() - p2.x();
    double y_offset = p1.y() - p2.y();
    double z_offset = p1.z() - p2.z();

    double distance = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2));

    int n = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2)) / step;
    int idx, idy, idz, idx_end, idy_end, idz_end;
    global_map->coord2gridIndex(p2_x, p2_y, p2_z, idx, idy, idz);
    global_map->coord2gridIndex(p1_x, p1_y, p1_z, idx_end, idy_end, idz_end);
    double last_height = p2_z;  ///起点高度

    // 如果两个点都在桥洞区域且高度差很多，说明一个在桥下一个在桥上，直接寄
    if(global_map->GridNodeMap[idx][idy]->exist_second_height &&
        global_map->GridNodeMap[idx_end][idy_end]->exist_second_height){

        if(abs(p1.z() - p2.z()) > 0.3){
            direction = 0;
            return false;
        }
    }

    // 采样查看是否存在高度差距过大的点
    for(int i = 0; i<(n+1); i++){
        ray_ptx = p2_x + i * step * x_offset / distance;
        ray_pty = p2_y + i * step * y_offset / distance;
        ray_ptz = 0.0;

        int pt_idx, pt_idy, pt_idz;
        global_map->coord2gridIndex(ray_ptx, ray_pty, ray_ptz, pt_idx, pt_idy, pt_idz);
        double height;
        if(!global_map->GridNodeMap[pt_idx][pt_idy]->exist_second_height) {
            height = global_map->getHeight(pt_idx, pt_idy);  //非桥洞区域正常通行
        }else{  // 进入桥洞区域后的高度不再根据bev更新，而是直接等于上一个点的高度。
            height = last_height;
            if(global_map->GridNodeMap[idx_end][idy_end]->exist_second_height){
                last_height = p1_z;
            }

        }

        if(abs(height - last_height) > 0.3){
            direction = 0;
            return false;
        }

        if (abs(height - last_height) > thresh){
            if(height > last_height){
                direction = 1;  // connector处于台阶下方
            }else{
                direction = 2;
            }
            return false;
        }
        last_height = height;
    }
    return true;
}


void TopoSearcher::checkDistanceFinalPath()
{  // 检查搜索出来的每一条路径的长度，进行排序，找到最佳路径
    distanceSet.clear();
    final_paths.clear();
    for(int i = 0; i < raw_paths.size(); i++){
        double distance = pathLength(raw_paths[i]);
        distanceSet.insert(std::make_pair(distance, raw_paths[i]));
    }

    for(std::multimap<double, std::vector<Eigen::Vector3d>>::iterator it1 = distanceSet.begin(); it1 !=distanceSet.end(); ++it1){
        ROS_INFO("[Topo] distance: %f", (it1)->first);
        final_paths.push_back((it1)->second);

        if(final_paths.size() > 0){break;}
    }
}

Eigen::Vector3d TopoSearcher::getSample()
{
    /**
     * @brief 采样点的生成
     * @param
     * @return 采样点
     */
    Eigen::Vector3d pt;
    if(global_map->topo_sample_map.empty()){
        // Fallback: random sample near robot when topo_sample_map is empty
        ROS_WARN_THROTTLE(5.0, "[Topo] topo_sample_map is empty, falling back to random local sampling");
        int x_i = int(m_rand_pos(m_eng) * 120);
        int y_i = int(m_rand_pos(m_eng) * 120);
        pt.x() = global_map->odom_position.x() + (x_i - 60) * 0.1;
        pt.y() = global_map->odom_position.y() + (y_i - 60) * 0.1;
        Eigen::Vector3i pt_idx = global_map->coord2gridIndex(pt);
        pt.z() = global_map->getHeight(pt_idx.x(), pt_idx.y());
        return pt;
    }
    if(m_rand_pos(m_eng) < 0.1){  // 30的概率采样附近的点
        for(int i = 0; i< 4 ; i++){
            int x_i = int(m_rand_pos(m_eng) * 120);
            int y_i = int(m_rand_pos(m_eng) * 120);
            pt.x() = global_map->odom_position.x() + (x_i - 60) * 0.1;
            pt.y() = global_map->odom_position.y() + (y_i - 60) * 0.1;
            Eigen::Vector3i pt_idx = global_map->coord2gridIndex(pt);
            pt.z() = global_map->getHeight(pt_idx.x(), pt_idx.y());
            if(!global_map->isOccupied(pt_idx, false)) {
                return pt;
            }
        }
    }
    int index = int(m_rand_pos(m_eng) * global_map->topo_sample_map.size());
    pt = global_map->topo_sample_map[index];
    return pt;
}

Eigen::Vector3d TopoSearcher::getLocalSample()
{
    /**
     * @brief 局部采样点的生成
     * @param
     * @return 采样点
     */
    Eigen::Vector3d pt;
    Eigen::Vector3d pt_inv;

    if(global_map->topo_sample_map.empty()){
        // Fallback: random sample in local ellipsoid when topo_sample_map is empty
        ROS_WARN_THROTTLE(5.0, "[Topo] topo_sample_map is empty in getLocalSample, falling back to random");
        pt(0) = (m_rand_pos(m_eng) * 2.0 - 1.0) * m_sample_r(0);
        pt(1) = (m_rand_pos(m_eng) * 2.0 - 1.0) * m_sample_r(1);
        pt = m_rotation * pt + m_translation;
        int pt_idx, pt_idy, pt_idz;
        global_map->coord2gridIndex(pt.x(), pt.y(), pt.z(), pt_idx, pt_idy, pt_idz);
        pt.z() = global_map->getHeight(pt_idx, pt_idy);
        return pt;
    }

    for(int i = 0; i < 10 ; i++){
        int index = int(m_rand_pos(m_eng) * global_map->topo_sample_map.size());
        pt = global_map->topo_sample_map[index];
        pt_inv = m_rotation_inv * (pt - m_translation);
        if(abs(pt_inv(0)) < m_sample_r(0) && abs(pt_inv(1)) < m_sample_r(1)){
            break;
        }
    }

    if(abs(pt_inv(0)) > m_sample_r(0) || abs(pt_inv(1)) > m_sample_r(1)){
        pt(0) = (m_rand_pos(m_eng) * 2.0 - 1.0) * m_sample_r(0);
        pt(1) = (m_rand_pos(m_eng) * 2.0 - 1.0) * m_sample_r(1);

        pt = m_rotation * pt + m_translation;

        int pt_idx, pt_idy, pt_idz;
        global_map->coord2gridIndex(pt.x(), pt.y(), pt.z(), pt_idx, pt_idy, pt_idz);
        pt.z() = global_map->getHeight(pt_idx, pt_idy);
    }

    return pt;
}

std::vector<GraphNode::Ptr> TopoSearcher::findMinVisibGuard(Eigen::Vector3d pt, int &exist_unvisiual){
    std::vector<GraphNode::Ptr> visib_guards;
    Eigen::Vector3d pt_temp;
    double temp_x = pt.x();
    double temp_y = pt.y();

    std::multimap<double, GraphNode::Ptr> MinDistanceSet;

    int visib_num = 0;
    for(std::vector<GraphNode::Ptr>::iterator iter = m_graph.begin(); iter != m_graph.end(); ++iter){
        double distance = std::sqrt(pow( temp_x - (*iter)->pos.x(), 2) +
                                    pow( temp_y - (*iter)->pos.y(), 2));  // 计算用于找到距离pt最近的点

        if((*iter)->type_ == GraphNode::Connector){  // 如果这个点距离太远或者是连接点就跳过
            continue;
        }
        if(distance > 5.0){
            continue;
        }

        if(lineVisib(pt, (*iter)->pos, 0.2, pt_temp, 0)){
            MinDistanceSet.insert(std::make_pair(distance, (*iter)));  // 返回所有能看到的点
        }
    }

    if(MinDistanceSet.size() < 2){
        return visib_guards;
    }else{
        visib_guards.push_back(MinDistanceSet.begin()->second);
        MinDistanceSet.erase(MinDistanceSet.begin());
        visib_guards.push_back(MinDistanceSet.begin()->second);
        return visib_guards;
    }
}
std::vector<GraphNode::Ptr> TopoSearcher::findVisibGuard(Eigen::Vector3d pt, double &dis_temp, int &exist_unvisiual)
{
    /**
     * @brief 根据采样点找可见的守卫点
     * @param pt 采样点
     * @param dis_temp 采样点到最近的守卫点的距离
     */
    std::vector<GraphNode::Ptr> visib_guards;
    Eigen::Vector3d pt_temp;
    double temp_x = pt.x();
    double temp_y = pt.y();

    double min_distance = 10000.0;

    int visib_num = 0;
    for(std::vector<GraphNode::Ptr>::iterator iter = m_graph.begin(); iter != m_graph.end(); ++iter)
    {
        double distance = std::sqrt(pow( temp_x - (*iter)->pos.x(), 2) +
                                    pow( temp_y - (*iter)->pos.y(), 2));  // 计算用于找到距离pt最近的点

        if((*iter)->type_ == GraphNode::Connector){  // 如果这个点距离太远或者是连接点就跳过
            continue;
        }
        if(distance > 5.0){
            continue;
        }

        if(distance < min_distance){
            min_distance = distance;
        }

        if(lineVisib(pt, (*iter)->pos, 0.2, pt_temp, 0)){
            visib_guards.push_back((*iter));
            ++visib_num;
            if (visib_num == 2)  // 如果存在两个可见的守卫点，判断是否需要连接，需要就直接break，否则扔掉其中一个点继续迭代，并把exist_unvisiual+1
            {
                bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
                if(need_connect){
                    break;
                }else{
                    visib_guards[0] = visib_guards[1];
                    visib_guards.pop_back();
                    visib_num --;
                    exist_unvisiual ++;
                }
            }
        }
    }
    dis_temp = min_distance;
    return visib_guards;
}

bool TopoSearcher::needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt)
{
    /**
     * @brief 判断两个点是否需要连接，即为两个点不管是单向可连或者双线可连，只要之前相互可见过就不再重新连接
     * @param g1 g2 两个判断的守卫点
     */
    for(int i = 0; i < g1->neighbors.size(); i++){
        for(int j = 0; j < g2->neighbors.size(); j++){
            if(g1->neighbors[i]->m_id == g2->neighbors[j]->m_id)  // 只要这俩连接过，我们就认为这段路径就不会重连接了
            {
                return false;
            }
        }
    }

//    for(int i = 0; i < g1->neighbors_but_noconnected.size(); i++){
//        for(int j = 0; j < g2->neighbors_but_noconnected.size(); j++){
//            if(g1->neighbors_but_noconnected[i]->m_id == g2->neighbors_but_noconnected[j]->m_id)
//                // 只要这俩相邻过，我们就认为这段路径就不会重连接了
//            {
//                return false;
//            }
//        }
//    }
//
//    for(int i = 0; i < g1->neighbors.size(); i++){
//        for(int j = 0; j < g2->neighbors_but_noconnected.size(); j++){
//            if(g1->neighbors[i]->m_id == g2->neighbors_but_noconnected[j]->m_id)  // 只要这俩相邻连接过，我们就认为这段路径就不会重连接了
//            {
//                return false;
//            }
//        }
//    }
//
//    for(int i = 0; i < g1->neighbors_but_noconnected.size(); i++){
//        for(int j = 0; j < g2->neighbors.size(); j++){
//            if(g1->neighbors_but_noconnected[i]->m_id == g2->neighbors[j]->m_id)  // 只要这俩**过，我们就认为这段路径就不会重连接了
//            {
//                return false;
//            }
//        }
//    }

    return true;
}

bool TopoSearcher::sameTopoPath(const std::vector<Eigen::Vector3d>& path1, const std::vector<Eigen::Vector3d>& path2, double thresh)
{   // 同样的topo路径就不放入m_graph中, 但是实际效果会增加计算量，而且用处也不是特别大，所以暂且放置
    double len1 = pathLength(path1);
    double len2 = pathLength(path2);

    double max_len = std::max(len1, len2);
    int pt_num = ceil(max_len / 0.2);

    std::vector<Eigen::Vector3d> pts1 = discretizePath(path1, pt_num);
    std::vector<Eigen::Vector3d> pts2 = discretizePath(path2, pt_num);

    Eigen::Vector3d pt_temp;
    for (int i = 0; i < pt_num; ++i) {
        if (!lineVisib(pts1[i], pts2[i], thresh, pt_temp, 1)) {
            return false;
        }
    }
    return true;
}

bool TopoSearcher::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh, Eigen::Vector3d& pc, int caster_id)
{
    double ray_ptx, ray_pty, ray_ptz;
    double p2_x, p2_y, p2_z;
    double distance_thresh;
    p2_x = p2.x();
    p2_y = p2.y();
    p2_z = p2.z();

    double x_offset = p1.x() - p2.x();
    double y_offset = p1.y() - p2.y();

    double distance = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2));

    int n = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2)) / thresh;
    if(caster_id == 1) {
        distance_thresh = 4.0;
    }

    if(distance < 0.4 && abs(p2_z - p1.z()) > 0.4){  //如果俩点太近且高度差太多直接认为不可见
        return false;
    }
    double last_height = p2_z;

    for(int i = 0; i<n+1; i++)
    {
        ray_ptx = p2_x + i * thresh * x_offset / distance;
        ray_pty = p2_y + i * thresh * y_offset / distance;
        ray_ptz = 0.0;

        int pt_idx, pt_idy, pt_idz;
        bool second_height = false;
        global_map->coord2gridIndex(ray_ptx, ray_pty, ray_ptz, pt_idx, pt_idy, pt_idz);
        double height = global_map->getHeight(pt_idx, pt_idy);
        if(global_map->GridNodeMap[pt_idx][pt_idy]->exist_second_height){
            height = last_height;
            if(height < 0.4){
                second_height = true;  // 在桥洞区域我们要判断这个点是不是在第二高度上
            }
            last_height = p1.z();
        }
        if(abs(height - last_height) > 0.4){  // 这样在只要产生了很大的距离差就直接返回false
            return false;
        }

        if (global_map->isOccupied(pt_idx, pt_idy, pt_idz, second_height)){
            return false;
        }
        last_height = height;
    }
    return true;
}

void TopoSearcher::pruneGraph()
{  // 懒得写了，先这样

}

double TopoSearcher::pathLength(const std::vector<Eigen::Vector3d> &path)
{
    double length = 0.0;
    if (path.size() < 2) return length;

    for (int i = 0; i < path.size() - 1; ++i) {
        length += (path[i + 1] - path[i]).norm();
    }
    return length;
}

std::vector<Eigen::Vector3d> TopoSearcher::discretizePath(std::vector<Eigen::Vector3d> path, int pt_num)
{
    double path_length = pathLength(path);  /// pt_num个点，pt_num-1段线段
    double resolution = path_length / (pt_num - 1);

    std::vector<Eigen::Vector3d> dis_path;
    Eigen::Vector3d head_pos = path.back();  // 第一个点
    path.pop_back();
    Eigen::Vector3d tail_pos = path.back();

    dis_path.push_back(head_pos);
    double temp_distance = 0.0;
    for(int i = 1; i < pt_num; i++)
    {
        double temp_distance_thresh = (tail_pos - head_pos).norm();
        temp_distance += resolution;
        if(temp_distance >= temp_distance_thresh)
        {
            path.pop_back();
            dis_path.push_back(tail_pos);
            head_pos = tail_pos;
            tail_pos = path.back();
            temp_distance = 0.0;
            continue;
        }
        Eigen::Vector3d direction = (tail_pos - head_pos).normalized();

        Eigen::Vector3d temp_pos = {head_pos.x() + resolution * i * direction.x(),
                                    head_pos.y() + resolution * i * direction.y(), 0.0 };
        dis_path.push_back(temp_pos);
    }

    return dis_path;

}


