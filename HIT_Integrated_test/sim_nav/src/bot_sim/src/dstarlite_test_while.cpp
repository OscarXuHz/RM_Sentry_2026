#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include<iostream>
#include<queue>
#include<stack>
#include<set>
#include<math.h>
#define INF 1e9
#define NEW 0
#define IN_LIST 1
#define OUT_LIST 2
ros::Publisher *pub_map;
ros::Publisher pub3;
class dstarlite{
    public:
        class Node;
        typedef Node* Nodeptr;
        void reset_previous_dynamic_map_info();
        void reset();
        // void astar_main(int start_x, int start_y, int goal_x, int goal_y);
        // void astar_update(Nodeptr end_node);
        // void astar_update_node(Nodeptr cur, double nf, Nodeptr succ);
        void dstar_main(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, double real_start_x, double real_start_y, std::string robot_frame_name, tf2_ros::Buffer& tfBuffer);
        void dstar_update(Nodeptr end_node);
        void dstar_update_node(Nodeptr cur, double nf, Nodeptr succ);
        void dstar_update_node(Nodeptr cur, Nodeptr succ);
        void dstar_update_node(Nodeptr cur);
        void publish(ros::Publisher& pub, std::string map_frame_name, ros::Publisher& cmd_vel_pub, std::string robot_frame_name, tf2_ros::Buffer& tfBuffer);
        void publish_vel(std::string map_frame_name, ros::Publisher& cmd_vel_pub, std::string robot_frame_name, tf2_ros::Buffer& tfBuffer);
        void publish_all_map_status(ros::Publisher& pub);
        void when_receive_new_goal(geometry_msgs::PointStamped::ConstPtr goal_pose_msg, double real_start_x, double real_start_y);
        void when_receive_new_dynamic_map(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, std::string map_frame_name, tf2_ros::Buffer& tfBuffer);
        void try_to_find_path();
        bool old_path_still_work(int start_x, int start_y);
        int from_real_x_to_map_x(double x){
            // ROS_INFO("from_real_x_to_map_x: %lf %lf %lf", x, initial_x, resolution);
            return (x - initial_x) / resolution;
        }
        int from_real_y_to_map_y(double y){
            // ROS_INFO("from_real_y_to_map_y: %lf %lf %lf", y, initial_y, resolution);
            return (y - initial_y) / resolution;
        }
        double calculate_velocity(Nodeptr cur);
        double calculate_edge_value(Nodeptr cur);
        dstarlite(std::string map_topic, double x0, double k, double L, double x01, double k1, double L1, double start_decrease_dis, double min_velocity_rate);
        ~dstarlite();
        class Node{
            public:
                double dis_to_goal, rhs;
                double dis_to_obstacle, edge_value;
                double obstacle_possibility, static_obstacle_possibility;
                bool reversed;
                int x, y, astar_list_status, dstar_list_status, appear_time;
                int last_out_list_round, round_for_find_path, round_for_dynamic_map;
                Nodeptr succ, father, son[2];
                Node(int x, int y){
                    dis_to_goal = INF + 1;
                    appear_time = 0;
                    rhs = INF;
                    this->obstacle_possibility = this->static_obstacle_possibility = 0;
                    this->x = x;
                    this->y = y;
                    astar_list_status = dstar_list_status = NEW;
                    last_out_list_round = round_for_dynamic_map = round_for_find_path = 0;
                    succ = father = son[0] = son[1] = nullptr;
                }
                Node(){}
                void clear(){
                    dis_to_goal = INF + 1;
                    rhs = INF;
                    obstacle_possibility = static_obstacle_possibility;
                    astar_list_status= dstar_list_status = NEW;
                    succ = father = son[0] = son[1] = nullptr;
                    appear_time = 0;
                    reversed = false;
                }
                class Compare_in_Dstar{
                    public:
                        bool operator()(Nodeptr a, Nodeptr b){
                            if(std::min(a->dis_to_goal, a->rhs) == std::min(b->dis_to_goal, b->rhs)){
                                if(a -> x == b -> x){
                                    return a -> y < b -> y;
                                }
                                return a -> x < b -> x;
                            }
                            return std::min(a->dis_to_goal, a->rhs) < std::min(b->dis_to_goal, b->rhs);
                        }
                };
                class Compare_in_dynamic_map{
                    public:
                        bool operator()(Nodeptr a, Nodeptr b){
                            if(a->round_for_dynamic_map == b->round_for_dynamic_map){
                                if(a->x == b->x){
                                    return a->y < b->y;
                                }
                                return a->x < b->x;
                            }
                            return a->round_for_dynamic_map < b->round_for_dynamic_map;
                        }
                };
        };
        class LinkCutTree{
            public:
                void reverse(Nodeptr now){now->reversed ^= 1;}
                bool get_son(Nodeptr x){return x->father->son[1] == x;}
                bool is_root(Nodeptr x){return x->father == nullptr || (x->father->son[0] != x && x->father->son[1] != x);}
                void rotate(Nodeptr x){
                    Nodeptr f = x->father, ff = f->father;
                    bool son = get_son(x);
                    if(!is_root(f)) ff->son[get_son(f)] = x; x -> father = ff;
                    f -> son[son] = x -> son[son^1]; if(x->son[son^1] != nullptr) x->son[son^1]->father = f;
                    x -> son[son^1] = f; f -> father = x;
                }
                void pushdown(Nodeptr now){
                    if(now->reversed){
                        now->reversed = false;
                        if(now->son[0] != nullptr) now->son[0]->reversed ^= 1;
                        if(now->son[1] != nullptr) now->son[1]->reversed ^= 1;
                        std::swap(now->son[0], now->son[1]);
                    }
                    return;
                }
                void splay(Nodeptr now){
                    std::stack<Nodeptr> s;
                    Nodeptr temp = now;
                    while(!is_root(temp)){
                        s.push(temp);
                        temp = temp->father;
                    }
                    s.push(temp);
                    while(!s.empty()){
                        pushdown(s.top());
                        s.pop();
                    }
                    while(!is_root(now)){
                        if(!is_root(now -> father)){
                            if(get_son(now->father) == get_son(now)) rotate(now->father);
                            else rotate(now);
                        }
                        rotate(now);
                    }
                }
                void access(Nodeptr now){
                    Nodeptr last = nullptr;
                    for(Nodeptr x = now; x != nullptr; x = x->father){
                        splay(x);
                        x->son[1] = last;
                        last = x;
                    }
                }
                void makeroot(Nodeptr now){
                    access(now);
                    splay(now);
                    reverse(now);
                }
                Nodeptr find_root(Nodeptr now){
                    access(now);
                    splay(now);
                    while(now->son[0] != nullptr) now = now->son[0];
                    return now;
                }
                void link(Nodeptr x, Nodeptr y){
                    // ROS_INFO("Link %d %d %d %d", x->x, x->y, y->x, y->y);
                    if(find_root(x) == find_root(y)){
                        // ROS_ERROR("Link Error");
                        // exit(1);
                        return ;
                    }
                    // ROS_INFO("Link %d %d %d %d", x->x, x->y, y->x, y->y);
                    makeroot(x);
                    makeroot(y);
                    x->father = y;
                }
                void del(Nodeptr x, Nodeptr y){
                    if(x == nullptr || y == nullptr)return;
                    makeroot(x);
                    access(y);
                    splay(y);
                    if(y->son[0] == x && x->son[1] == nullptr && x->son[0] == nullptr){ 
                        y->son[0] = nullptr;
                        x->father = nullptr;
                    }
                }
        };
        LinkCutTree* lct;
        Nodeptr final_goal_node, start_node, origin_start_node;
        int max_x, max_y, round, for_find_path_round, for_dynamic_map_round;
        Nodeptr** map;
        // std::set<Nodeptr, Node::Compare_in_Astar> astar_list;
        std::set<Nodeptr, Node::Compare_in_Dstar> dstar_list;
        std::set<Nodeptr, Node::Compare_in_dynamic_map> changed_obstacle_nodes;
        int dx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
        int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};
        ros::Rate* rate_for_straight_line, *rate_for_diagonal_line;
        double velocity, resolution, initial_x, initial_y;
        double k, x0, L, k1, x01, L1;
        double start_decrease_dis, min_velocity_rate;
};
dstarlite::dstarlite(std::string map_topic, double x0, double k, double L, double x01, double k1, double L1, double start_decrease_dis, double min_velocity_rate){
    ros::NodeHandle nh;
    nav_msgs::OccupancyGrid::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, nh);
    ROS_INFO("get_map_started");
    if (msg != nullptr){
        round = 0;
        max_x = msg->info.width;
        max_y = msg->info.height;
        resolution = msg->info.resolution;
        initial_x = msg->info.origin.position.x;
        initial_y = msg->info.origin.position.y;
        for_dynamic_map_round = 0;
        this->x0 = x0;
        this->k = k;
        this->L = L; 
        this->x01 = x01;
        this->k1 = k1;
        this->L1 = L1;
        this->start_decrease_dis = start_decrease_dis;
        this->min_velocity_rate = min_velocity_rate;
        this->origin_start_node = nullptr;
        this->start_node = nullptr;
        this->final_goal_node = nullptr;
        //needcode to initialize max_x and max_y;
        map = new Nodeptr*[max_x];
        std::queue<Nodeptr> q;
        for(int i = 0; i < max_x; i++){
            map[i] = new Nodeptr[max_y];
            for(int j = 0; j < max_y; j++){
                map[i][j] = new Node(i, j);
                if(msg->data[i + j * max_x] == 100){
                    map[i][j]->static_obstacle_possibility = map[i][j]->obstacle_possibility = 100;
                    q.push(map[i][j]);
                }
            }
        }
        double decrease_rate = 5;
        while(!q.empty()){
            Nodeptr cur = q.front();
            q.pop();
            for(int i = 0; i < 8; i++){
                int nx = cur->x + dx[i];
                int ny = cur->y + dy[i];
                if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                if(map[nx][ny]->obstacle_possibility < cur->obstacle_possibility - (i < 4 ? 1.0 : 1.414) * decrease_rate){
                    map[nx][ny]->static_obstacle_possibility = map[nx][ny]->obstacle_possibility = cur->obstacle_possibility - (i < 4 ? 1.0 : 1.414) * decrease_rate;
                    q.push(map[nx][ny]);
                }
            }
        }
        rate_for_straight_line = new ros::Rate(20);
        rate_for_diagonal_line = new ros::Rate(20*1.414);
        lct = new LinkCutTree();
        //can be further developed by changing the refreshing rate
    }
    else{
        ROS_ERROR("Failed to get map");
        return;
    }
    ROS_INFO("get_map_finished");
}
dstarlite::~dstarlite(){
    for(int i = 0; i < max_x; i++){
        for(int j = 0; j < max_y; j++){
            delete map[i][j];
        }
        delete[] map[i];
    }
    delete rate_for_straight_line;
    delete rate_for_diagonal_line;
    delete[] map;
    delete lct;
}
void dstarlite::when_receive_new_goal(geometry_msgs::PointStamped::ConstPtr goal_pose_msg, double real_start_x, double real_start_y){
    int start_x, start_y, goal_x, goal_y;
    ROS_INFO("goal_x: %lf goal_y: %lf start_x: %lf start_y: %lf resolution: %lf", goal_pose_msg->point.x, goal_pose_msg->point.y, real_start_x, real_start_y, resolution);
    ROS_INFO("get_real_goal_info");
    goal_x = from_real_x_to_map_x(goal_pose_msg->point.x);
    goal_y = from_real_y_to_map_y(goal_pose_msg->point.y);
    ROS_INFO("get_real_start_info");
    if(final_goal_node != nullptr&&final_goal_node->x == goal_x && final_goal_node->y == goal_y)return;
    ROS_INFO("get_real_start_info_start");

    start_x = from_real_x_to_map_x(real_start_x);
    start_y = from_real_y_to_map_y(real_start_y);
    if(start_x < 0 || start_x >= max_x || start_y < 0 || start_y >= max_y){
        ROS_ERROR("Out Of Map!!!");
        start_node = nullptr;
        return;
    }
    final_goal_node = map[goal_x][goal_y];
    origin_start_node = start_node = map[start_x][start_y];
    ROS_INFO("Clear Map");
    reset();
    dstar_list.clear();
    changed_obstacle_nodes.clear();
    ROS_INFO("clear finished");
    // for(int i = 0; i < max_x; i++)for(int j = 0; j < max_y; j++)map[i][j]->manhattan_dis_to_start = abs(i - start_x) + abs(j - start_y);
    // map[goal_x][goal_y]->dis_to_goal = 0;
    map[goal_x][goal_y]->rhs = 0;
    dstar_list.insert(map[goal_x][goal_y]);
    map[goal_x][goal_y]->dstar_list_status = IN_LIST;
}
void dstarlite::when_receive_new_dynamic_map(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, std::string map_frame_name, tf2_ros::Buffer& tfBuffer){
    int x_size_for_dynamic_map = dynamic_map_msg->info.width;
    int y_size_for_dynamic_map = dynamic_map_msg->info.height;
    double resolution_for_dynamic_map = dynamic_map_msg->info.resolution;
    double initial_x_for_dynamic_map = dynamic_map_msg->info.origin.position.x;
    double initial_y_for_dynamic_map = dynamic_map_msg->info.origin.position.y;
    std::string dynamic_frame_name = dynamic_map_msg->header.frame_id;
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(map_frame_name, dynamic_frame_name, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Erase obstacle nodes");
    ROS_INFO("Deal with dynamic map");
    for_dynamic_map_round++;
    for(int i = 0; i < x_size_for_dynamic_map; i++){
        for(int j = 0; j < y_size_for_dynamic_map; j++){
            int index = i + j * x_size_for_dynamic_map;
            // if (dynamic_map_msg->data[index] == 100){
            double x_in_dynamic_map = i * resolution_for_dynamic_map + initial_x_for_dynamic_map;
            double y_in_dynamic_map = j * resolution_for_dynamic_map + initial_y_for_dynamic_map;
            tf2::Vector3 dynamic_map_vector = tf2::Vector3(x_in_dynamic_map, y_in_dynamic_map, 0);
            tf2::Transform tf_transform;
            tf2::fromMsg(transformStamped.transform, tf_transform);
            tf2::Vector3 map_vector = tf_transform * dynamic_map_vector;
            // ROS_INFO("dstar node: %lf %lf %d %d", map_vector.x(), map_vector.y(), x, y);
            int x = from_real_x_to_map_x(map_vector.x());
            int y = from_real_y_to_map_y(map_vector.y());
            // ROS_INFO("dstar node: %lf %lf %d %d", map_vector.x(), map_vector.y(), x, y);
            if(x < 0 || x >= max_x || y < 0 || y >= max_y || dynamic_map_msg->data[index] == -1)continue;
            changed_obstacle_nodes.erase(map[x][y]);
            map[x][y]->round_for_dynamic_map = for_dynamic_map_round;
            changed_obstacle_nodes.insert(map[x][y]);
            // if(dynamic_map_msg->data[index] != map[x][y]->obstacle_possibility && dynamic_map_msg->data[index] >= map[x][y]->static_obstacle_possibility){
            map[x][y]->obstacle_possibility = std::max((double)dynamic_map_msg->data[index], map[x][y]->static_obstacle_possibility);
            // ROS_INFO("map:[%d][%d]->obstacle_possibility=%lf",x,y ,map[x][y]->obstacle_possibility);
            dstar_update_node(map[x][y]);
            // }
        }
    }
    while(!changed_obstacle_nodes.empty()){
        Nodeptr cur = *changed_obstacle_nodes.begin();
        if(cur->round_for_dynamic_map + 60 <= for_dynamic_map_round){
            changed_obstacle_nodes.erase(cur);
            cur->obstacle_possibility = cur->static_obstacle_possibility;
            dstar_update_node(cur);
        }
        else break;
    }
}
void dstarlite::reset(){for(int i = 0; i < max_x; i++)for(int j = 0; j < max_y; j++)map[i][j]->clear();}
void dstarlite::dstar_update_node(Nodeptr cur){
    if(cur == final_goal_node)return;
    int previous_list_status = cur->dstar_list_status;
    if(cur->dstar_list_status == IN_LIST){
        dstar_list.erase(cur);
        cur->dstar_list_status = OUT_LIST;
    }
    Nodeptr previous_succ = cur->succ;
    double previous_rhs = cur->rhs;
    double ox = 0, oy = 0, min_acceptable_val = 0;
    cur->rhs = INF;
    cur->succ = nullptr;
    if(previous_succ != nullptr){
        ox = previous_succ->x;
        oy = previous_succ->y;
        min_acceptable_val = 0.1;
    }
    for(int i = 0; i < 8; i++){
        int nx = cur->x + dx[i];
        int ny = cur->y + dy[i];
        if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
        double value = calculate_edge_value(cur)*calculate_edge_value(map[nx][ny]);
        if(value < 1)ROS_ERROR("sfdassglkhaekl;erjtskl;jkl;m;lsdsg");
        double new_rhs = i<4?map[nx][ny]->dis_to_goal + value : map[nx][ny]->dis_to_goal + 1.414 * value;
        if(new_rhs + sqrt((nx - ox) * (nx - ox) + (ny - oy) * (ny - oy) * value) * min_acceptable_val < cur->rhs){
            cur->rhs = new_rhs;
            cur->succ = map[nx][ny];
        }
    }
    if(previous_rhs == cur -> dis_to_goal && previous_succ != nullptr)lct->del(cur, previous_succ);
    if(cur->rhs == cur->dis_to_goal && cur->succ != nullptr)lct->link(cur, cur->succ);
    if(cur -> rhs != cur->dis_to_goal){    
        cur->dstar_list_status = IN_LIST;
        dstar_list.insert(cur);
    }
}

void dstarlite::dstar_main(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, double real_start_x, double real_start_y, std::string map_frame_name, tf2_ros::Buffer& tfBuffer){
    int start_x = from_real_x_to_map_x(real_start_x);
    int start_y = from_real_y_to_map_y(real_start_y);
    if(start_x < 0 || start_y < 0 || start_x >= max_x || start_y >= max_y){
        ROS_ERROR("Out Of Map!!!");
        start_node = nullptr;
        return;
    }
    start_node = map[start_x][start_y];
    // if(start_node->isObstacle || final_goal_node->isObstacle){
    //     ROS_ERROR("Start or final node is obstacle");
    //     // return;
    // }
    if(old_path_still_work(start_x, start_y))return;
    ROS_INFO("start_pos: %d %d", start_x, start_y);
    dstar_update(map[start_x][start_y]);
    ROS_INFO("dstar_update_finished");
}
void dstarlite::dstar_update(Nodeptr end_node){
    ROS_INFO("dstar_list size: %d", dstar_list.size());
    int count = 0;
    round++;
    // ROS_INFO("origin_start_node_status: %lf %lf %d %lf", end_node->dis_to_goal, end_node->rhs, end_node->dstar_list_status, end_node->obstacle_possibility);
    // ros::Duration(1.0).sleep();
    while(!dstar_list.empty()){
        // round++;
        count++;
        Nodeptr cur = *dstar_list.begin();
        dstar_list.erase(cur);
        cur->dstar_list_status = OUT_LIST;
        if(cur->last_out_list_round != round)cur->appear_time = 0;
        cur->appear_time++;
        cur->last_out_list_round = round;
        // ROS_INFO("Current_node %d %d %lf %lf %d %d", cur->x, cur->y, cur->dis_to_goal, cur->rhs, final_goal_node->x, final_goal_node->y);
        // ros::Duration(0.1).sleep();
        if(cur->dis_to_goal < cur -> rhs){
            int x = cur->x, y = cur->y;
            cur -> dis_to_goal = INF + 1;
            cur -> dstar_list_status = IN_LIST;
            dstar_list.insert(cur);
            for(int i = 0; i < 8; i++){
                int nx = x + dx[i];
                int ny = y + dy[i];
                if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                dstar_update_node(map[nx][ny]);
            }
        }
        else if(cur -> dis_to_goal > cur -> rhs){
            int x = cur->x, y = cur->y;
            cur -> dis_to_goal = cur -> rhs;
            for(int i = 0; i < 8; i++){
                int nx = x + dx[i];
                int ny = y + dy[i];
                if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                dstar_update_node(map[nx][ny]);
            }
            if(cur->succ != nullptr)lct->link(cur, cur->succ);
        }
        if(lct->find_root(final_goal_node) == lct->find_root(end_node))break;
        // if(cur == end_node && cur->dstar_list_status == OUT_LIST)break;
        // if(end_node->dis_to_goal == end_node->rhs)break;
        // if(count%10000 == 0){
        //     try_to_find_path();
        //     publish_all_map_status(*pub_map);
        //     ROS_INFO("start_node_status: %lf %lf %d %lf %d %d %p", end_node->dis_to_goal, end_node->rhs, end_node->dstar_list_status, end_node->obstacle_possibility, round, end_node->last_out_list_round, end_node->succ);
        //     ROS_INFO("end_node_status: %lf %lf %lf %d %d %d %p", final_goal_node->dis_to_goal, final_goal_node->rhs, final_goal_node->obstacle_possibility, final_goal_node->dstar_list_status, round, final_goal_node->last_out_list_round, final_goal_node->succ);
        //     // ros::Duration(1).sleep();
        // }
    }
    // publish_all_map_status(*pub_map);
    ROS_INFO("dstar_list size end with: %d dstar_list update: %d", dstar_list.size(), count);
    // if(!dstar_list.empty()){
    //     ROS_ERROR("dstar_list is not empty");
    //     exit(1);
    // }
}

/*
void dstarlite::publish(ros::Publisher& pub, std::string map_frame_name, ros::Publisher& cmd_vel_pub, std::string robot_frame_name, tf2_ros::Buffer& tfBuffer){
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = map_frame_name; 
    // ROS_INFO("Publish started");
    Nodeptr cur = start_node;
    if(cur == nullptr)return;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(robot_frame_name, map_frame_name, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    // ROS_INFO("Find root");
    if(cur == nullptr || final_goal_node == nullptr) ROS_ERROR("?????");
    if(lct->find_root(cur) == lct->find_root(final_goal_node)){
        // ROS_INFO("cur->succ:%p cur:%p root(cur)%p goal%p root(goal)%p", cur->succ, cur, lct->find_root(cur), final_goal_node, lct->find_root(final_goal_node));
        Nodeptr final_aim = cur;
        for(int i = 0; i < 12; i++){
            if(final_aim->succ == nullptr)break;
            final_aim = final_aim->succ;
        }
        double dx = final_aim->x - cur->x;
        double dy = final_aim->y - cur->y;
        double dis = sqrt(dx * dx + dy * dy);
        double new_velocity = calculate_velocity(cur);
        cmd_vel.linear.x = dx / dis;
        cmd_vel.linear.y = dy / dis;
        tf2::Vector3 old_cmd_vel = tf2::Vector3(cmd_vel.linear.x, cmd_vel.linear.y, 0);
        tf2::Transform tf_transform;
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0;
        tf2::fromMsg(transformStamped.transform, tf_transform);
        tf2::Vector3 new_cmd_vel = tf_transform * old_cmd_vel;
        double xy_lenth = sqrt(new_cmd_vel.x() * new_cmd_vel.x() + new_cmd_vel.y() * new_cmd_vel.y());
        double z_angle = atan2(new_cmd_vel.z(), xy_lenth);
        cmd_vel.linear.x = new_cmd_vel.x()/xy_lenth * new_velocity * (std::min(1 - z_angle/(15.0 / 360 * 2 * 3.1415926), 1.7));
        cmd_vel.linear.y = new_cmd_vel.y()/xy_lenth * new_velocity * (std::max(1 - z_angle/(15.0 / 360 * 2 * 3.1415926), 0.5));
        ROS_INFO("old_cmd_vel: %lf new_cmd_vel: %lf z_angle:%lf", sqrt(old_cmd_vel.x()*old_cmd_vel.x()+old_cmd_vel.y()*old_cmd_vel.y()), sqrt(cmd_vel.linear.x*cmd_vel.linear.x+cmd_vel.linear.y*cmd_vel.linear.y), z_angle*180/3.1415926);
        cmd_vel_pub.publish(cmd_vel);
    }
    else{
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel_pub.publish(cmd_vel);
        return;
    }
    // ROS_INFO("Find path");
    int count = 0;
    while(cur != nullptr && cur != final_goal_node){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = cur->x * resolution + initial_x;
        pose.pose.position.y = cur->y * resolution + initial_y;
        pose.pose.position.z = 0;
        path.poses.push_back(pose);
        // count++;
        // if(count < 100)ROS_INFO("path: %lf %lf %d %d %lf %lf %d %d %d %d", pose.pose.position.x, pose.pose.position.y, cur->x, cur->y, cur->dis_to_goal, cur->rhs, cur->last_out_list_round, round, cur->dstar_list_status, (int)cur->isObstacle) ;
        cur = cur->succ;
    }
    pub.publish(path);
    // ROS_INFO("Publish finished");
}*/

/*
    *This part of code is for testing TEB by ROS
*/
//new version of publish function
void dstarlite::publish(ros::Publisher& pub, std::string map_frame_name, ros::Publisher& cmd_vel_pub, std::string robot_frame_name, tf2_ros::Buffer& tfBuffer) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = map_frame_name;

    Nodeptr cur = start_node; // 或 this->start_node
    if (cur == nullptr) return;

    while (cur != nullptr && cur != final_goal_node) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = cur->x * resolution + initial_x;
        pose.pose.position.y = cur->y * resolution + initial_y;
        pose.pose.position.z = 0;
        path.poses.push_back(pose);
        cur = cur->succ;
    }

    pub.publish(path);
}

void publishObstacles(const nav_msgs::OccupancyGrid::ConstPtr& dynamic_map_msg, ros::Publisher& obstacle_pub) {
    costmap_converter::ObstacleArrayMsg obstacles;
    obstacles.header.stamp = ros::Time::now();
    obstacles.header.frame_id = dynamic_map_msg->header.frame_id;

    int width = dynamic_map_msg->info.width;
    int height = dynamic_map_msg->info.height;
    double resolution = dynamic_map_msg->info.resolution;
    double origin_x = dynamic_map_msg->info.origin.position.x;
    double origin_y = dynamic_map_msg->info.origin.position.y;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            if (dynamic_map_msg->data[index] == 100) {  // 障碍物
                costmap_converter::ObstacleMsg obstacle;
                geometry_msgs::Point32 pt;
                pt.x = x * resolution + origin_x;
                pt.y = y * resolution + origin_y;
                pt.z = 0.0;
                obstacle.polygon.points.push_back(pt);
                obstacle.id = index;
                obstacles.obstacles.push_back(obstacle);
            }
        }
    }

    obstacle_pub.publish(obstacles);
}

void dstarlite::publish_all_map_status(ros::Publisher& pub){
    nav_msgs::OccupancyGrid all_map_status;
    all_map_status.header.stamp = ros::Time::now();
    all_map_status.header.frame_id = "map";
    all_map_status.info.width = max_x;
    all_map_status.info.height = max_y;
    all_map_status.info.resolution = resolution;
    all_map_status.info.origin.position.x = initial_x;
    all_map_status.info.origin.position.y = initial_y;
    all_map_status.info.origin.position.z = 0;
    all_map_status.info.origin.orientation.x = 0;
    all_map_status.info.origin.orientation.y = 0;
    all_map_status.info.origin.orientation.z = 0;
    all_map_status.info.origin.orientation.w = 1;
    all_map_status.data.resize(max_x * max_y);
    int count = 0;
    for(int i = 0; i < max_x; i++){
        for(int j = 0; j < max_y; j++){
            int index = i + j * max_x;
            // if(map[i][j]->isObstacle)all_map_status.data[index] = 100;
            // if(map[i][j]->last_out_list_round == round){
            //     all_map_status.data[index] = 100;
            //     count++;
            // }
            // else all_map_status.data[index] = 0;
            all_map_status.data[index] = map[i][j]->obstacle_possibility;
        }
    }
    ROS_ERROR("count: %d", count);
    pub.publish(all_map_status);

}
bool dstarlite::old_path_still_work(int start_x, int start_y){
    Nodeptr cur = map[start_x][start_y];
    return lct->find_root(cur) == lct->find_root(final_goal_node);
}
bool get_dynamic_map_info = false;
nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg;
void record_dynamic_map_info(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    dynamic_map_msg = msg;    
    get_dynamic_map_info = true;
    // ROS_INFO("get dynamic map info");
}
void dstarlite::try_to_find_path(){
    Nodeptr cur = start_node;
    // ROS_INFO("%lf %lf %d", cur->x, cur->y, cur);
    int count = 0;
    for_find_path_round++;
    ROS_INFO("try_to_find_path");
    while(cur!=nullptr && cur!=final_goal_node){
        if(cur->round_for_find_path == for_find_path_round){
            ROS_ERROR("Loop for goal");
            return;
        }
        if(cur->succ == nullptr){
            ROS_ERROR("not updated");
            break;
        }
        if((lct->find_root(cur) != lct->find_root(cur->succ) && cur->dis_to_goal == cur->rhs && cur->succ->dis_to_goal == cur->succ->rhs)){
            ROS_ERROR("Not connected");
            ros::Duration(4).sleep();   
        }
        cur->round_for_find_path = for_find_path_round;
        ROS_INFO("%d %d %d %lf %lf", cur->x, cur->y, cur->last_out_list_round, cur->dis_to_goal, cur->rhs);
        cur = cur->succ;
        count++;
    }
    if(cur == final_goal_node)ROS_INFO("Find path");

    return;
}
double dstarlite::calculate_velocity(Nodeptr cur){
    double velocity = L1/(1 + exp(-k1 * (cur->obstacle_possibility - x01)));
    if(sqrt((cur->x - final_goal_node->x) * (cur->x - final_goal_node->x) + (cur->y - final_goal_node->y) * (cur->y - final_goal_node->y))*resolution < start_decrease_dis)
        velocity*=min_velocity_rate+(1-min_velocity_rate)/start_decrease_dis*sqrt((cur->x - final_goal_node->x) * (cur->x - final_goal_node->x) + (cur->y - final_goal_node->y) * (cur->y - final_goal_node->y))*resolution;
    return L1;
}
double dstarlite::calculate_edge_value(Nodeptr cur){
    return std::max(1 + L/(1 + exp(-k * (cur->static_obstacle_possibility - x0))), (1 + L/(1 + exp(-k * (cur->obstacle_possibility - x0))))*0.5);
}
geometry_msgs::PointStamped::ConstPtr goal_pose_msg;
bool get_goal_info = false;
void record_goal_info(const geometry_msgs::PointStamped::ConstPtr& msg){
    goal_pose_msg = msg;
    get_goal_info = true;
    ROS_INFO("get goal info");
}
/*
*Test for TEB
*/
void tebCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    pub3.publish(*msg);  // Publish TEB cmd
}

int main(int argc, char **argv){
    std::string node_name = "dstarlite";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    std::string static_map_topic_name;
    if (!nh.getParam(node_name+"/"+"map_topic_name", static_map_topic_name)) ROS_ERROR("Failed to get param 'map_topic_name'");
    ROS_INFO("static_map_topic_name: %s", static_map_topic_name.c_str());
    
    std::string map_frame_name;
    if (!nh.getParam(node_name+"/"+"map_frame_name", map_frame_name)) ROS_ERROR("Failed to get param 'map_frame_name'");
    ROS_INFO("map_frame_name: %s", map_frame_name.c_str());
    
    std::string robot_frame_name;
    if (!nh.getParam(node_name+"/"+"robot_frame_name", robot_frame_name)) ROS_ERROR("Failed to get param 'robot_frame_name'");
    ROS_INFO("robot_frame_name: %s", robot_frame_name.c_str());
    
    std::string dynamic_map_topic_name;
    if (!nh.getParam(node_name+"/"+"dynamic_map_topic_name", dynamic_map_topic_name)) ROS_ERROR("Failed to get param 'dynamic_map_topic_name'");
    ros::Subscriber dynamic_map_sub = nh.subscribe(dynamic_map_topic_name, 1, record_dynamic_map_info);
    ROS_INFO("dynamic_map_topic_name: %s", dynamic_map_topic_name.c_str());
    
    std::string goal_topic_name;
    if (!nh.getParam(node_name+"/"+"goal_topic_name", goal_topic_name)) ROS_ERROR("Failed to get param 'goal_topic_name'");
    ros::Subscriber goal_sub = nh.subscribe(goal_topic_name, 1, record_goal_info);
    ROS_INFO("goal_topic_name: %s", goal_topic_name.c_str());
    
    double x0_grid;
    if (!nh.getParam(node_name+"/"+"x0_grid", x0_grid)) ROS_ERROR("Failed to get param 'x0_grid'");
    ROS_INFO("x0_grid: %lf", x0_grid);

    double k_grid;
    if (!nh.getParam(node_name+"/"+"k_grid", k_grid)) ROS_ERROR("Failed to get param 'k_grid'");
    ROS_INFO("k_grid: %lf", k_grid);

    double L_grid;
    if (!nh.getParam(node_name+"/"+"L_grid", L_grid)) ROS_ERROR("Failed to get param 'L_grid'");
    ROS_INFO("L_grid: %lf", L_grid);

    double x0_velocity;
    if (!nh.getParam(node_name+"/"+"x0_velocity", x0_velocity)) ROS_ERROR("Failed to get param 'x0_velocity'");
    ROS_INFO("x0_velocity: %lf", x0_velocity);

    double k_velocity;
    if (!nh.getParam(node_name+"/"+"k_velocity", k_velocity)) ROS_ERROR("Failed to get param 'k_velocity'");
    ROS_INFO("k_velocity: %lf", k_velocity);

    double L_velocity;
    if (!nh.getParam(node_name+"/"+"L_velocity", L_velocity)) ROS_ERROR("Failed to get param 'L_velocity'");
    ROS_INFO("L_velocity: %lf", L_velocity);

    double start_decrease_dis;
    if (!nh.getParam(node_name+"/"+"start_decrease_dis", start_decrease_dis)) ROS_ERROR("Failed to get param 'start_decrease_dis'");
    ROS_INFO("start_decrease_dis: %lf", start_decrease_dis);

    double min_velocity_rate;
    if (!nh.getParam(node_name+"/"+"min_velocity_rate", min_velocity_rate)) ROS_ERROR("Failed to get param 'min_velocity_rate'");
    ROS_INFO("min_velocity_rate: %lf", min_velocity_rate);

    ros::Publisher pub = nh.advertise<nav_msgs::Path>("dstar_path", 1);
    ros::Publisher pub2 = nh.advertise<nav_msgs::OccupancyGrid>("all_map_status", 1);
    pub3 = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher pub4 = nh.advertise<std_msgs::Bool>("dstar_status", 1);
    pub_map = &pub2;

    /*
    *This part of code is for testing TEB by ROS
    */
    ros::Publisher teb_path_pub;
    ros::Subscriber teb_cmd_vel_sub;
    teb_path_pub = nh.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/via_points", 1);
    ros::Publisher obstacle_pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("/move_base/TebLocalPlannerROS/obstacles", 1);
    teb_cmd_vel_sub = nh.subscribe("/cmd_vel", 1, tebCmdVelCallback);
    /*
    *---
    */

    ROS_INFO("Initialization started");
    dstarlite dstar(static_map_topic_name, x0_grid, k_grid, L_grid, x0_velocity, k_velocity, L_velocity, start_decrease_dis, min_velocity_rate);
    ROS_INFO("Initialization finished");
    bool have_first_goal = false;
    ros::Rate rate(30);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    while(ros::ok){
        // ROS_INFO("loop");
        ROS_INFO("D*teb start");
        /*
            *This part of code is for testing TEB by ROS
        */
        if(get_goal_info && get_dynamic_map_info){
            try {
                transformStamped = tfBuffer.lookupTransform(map_frame_name, robot_frame_name, ros::Time(0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            ROS_INFO("get goal and dynamic map info");
            dstar.when_receive_new_goal(goal_pose_msg, transformStamped.transform.translation.x, transformStamped.transform.translation.y);
            // dstar.dstar_main(dynamic_map_msg, transformStamped.transform.translation.x, transformStamped.transform.translation.y, map_frame_name, tfBuffer);
            // dstar.publish(transformStamped.transform.translation.x, transformStamped.transform.translation.y, pub, map_frame_name);
            get_goal_info = false;
            have_first_goal = true;
        }
        if(have_first_goal && get_dynamic_map_info){
            ROS_INFO("get dynamic map info");
            ROS_INFO("start_deal_with_dynamic_map");
            dstar.when_receive_new_dynamic_map(dynamic_map_msg, map_frame_name, tfBuffer);

            // 发布障碍物信息给 TEB
            publishObstacles(dynamic_map_msg, obstacle_pub);
            ROS_INFO("Publishing obstacles to TEB");

            ROS_INFO("finish");
            get_dynamic_map_info = false;
        }
        if(have_first_goal){
            try {
                transformStamped = tfBuffer.lookupTransform(map_frame_name, robot_frame_name, ros::Time(0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ROS_INFO("ERROR IN MAP TO ROBOT");
                ros::Duration(1.0).sleep();
            }
            double real_goal_x = dstar.final_goal_node->x * dstar.resolution + dstar.initial_x;
            double real_goal_y = dstar.final_goal_node->y * dstar.resolution + dstar.initial_y;
            if(abs(real_goal_x - transformStamped.transform.translation.x) < 0.2 && abs(real_goal_y - transformStamped.transform.translation.y) < 0.2){
                ROS_INFO("Goal reached");
                have_first_goal = false;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                pub3.publish(cmd_vel);
                std_msgs::Bool dstar_status;
                dstar_status.data = 1;
                pub4.publish(dstar_status);
                continue;
            }
            dstar.dstar_main(dynamic_map_msg, transformStamped.transform.translation.x, transformStamped.transform.translation.y, map_frame_name, tfBuffer);
            dstar.publish(pub, map_frame_name, pub3, robot_frame_name, tfBuffer);

            /*
                *This part of code is for testing TEB by ROS
            */
            // 将路径传递给 TEB
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = map_frame_name;
            dstarlite::Nodeptr cur = dstar.start_node;
            while (cur != nullptr && cur != dstar.final_goal_node) {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = cur->x * dstar.resolution + dstar.initial_x;
                pose.pose.position.y = cur->y * dstar.resolution + dstar.initial_y;
                path.poses.push_back(pose);
                cur = cur->succ;
            }
            // 检查路径是否为空
            if (path.poses.empty()) {
                ROS_WARN("Path is empty, stopping robot");
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                pub3.publish(cmd_vel);  // 停止机器人
            } else {
                teb_path_pub.publish(path);  // 发布路径给 TEB
                ROS_INFO("TEB path published with %zu poses", path.poses.size());
            }
            /*
            *---
            */
        }
        else{
            ROS_INFO("No goal");
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            pub3.publish(cmd_vel);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}