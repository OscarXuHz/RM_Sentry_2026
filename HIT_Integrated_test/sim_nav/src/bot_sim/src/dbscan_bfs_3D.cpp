#pragma once
#include <chrono>
#include <queue>
#include "ros/ros.h"
#include <iostream>
#include <fstream>   
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <ros/ros.h> 
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <tf/transform_datatypes.h>
#include <deque>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace std;
#define INF 1e9
//雷达参数 theta_square = 0.000279328; //theta = 0.0167131

extern int max_radar_x;
extern int max_radar_y;
extern int min_radar_x;
extern int min_radar_y;
int obstacle_enlargement=4;
int MAXN=100;
//bfs方向数组
int dx[4] = {0, 0, 1, -1};
int dy[4] = {1, -1, 0, 0};
int dx2[4] = {1, -1, 1, -1};
int dy2[4] = {1, 1, -1, -1};

//dbscan 点
struct dbscan_Point {
    float x, y, z;
    int cluster = 0;
    bool visited = false;
    bool noise = false;
};
//整数点
struct int_Point {
    int x, y;
};

//double距离
double dbscan_distance(dbscan_Point a, dbscan_Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

//聚类函数
void dbscan(vector<dbscan_Point>& data, double eps, int minPts,int& clusterId) {
    clusterId=1;
    for (dbscan_Point& p : data) { 
        if (p.visited) continue;
        p.visited = true;

        vector<dbscan_Point*> neighbors;
        for (dbscan_Point& q : data) {
            if (dbscan_distance(p, q) <= eps) {
                neighbors.push_back(&q);
            }
        }

        if (neighbors.size() < minPts) {
            p.noise = true;
        } 
        else {
            clusterId++;
            p.cluster = clusterId;

            for (dbscan_Point* q : neighbors) {
                if (!q->visited) {
                    q->visited = true;
                    vector<dbscan_Point*> neighbors2;
                    for (dbscan_Point& r : data) {
                        if (dbscan_distance(*q, r) <= eps) {
                            // cout<<dbscan_distance(*q, r)<<"\n";
                            neighbors2.push_back(&r);
                        }
                    }
                    if (neighbors2.size() >= minPts) {
                        for (dbscan_Point* r : neighbors2) {
                            if (r->cluster == 0) {
                                r->cluster = clusterId;
                            }
                        }
                    }
                }
                if (q->cluster == 0) {
                    q->cluster = clusterId;
                }
            }
        }
    }
}

//bfs膨胀
void bfs(vector<dbscan_Point>& points, vector<vector<double> >& data, int width, int height, int decrease) {
    queue<pair<int_Point, double> > q;
    for (dbscan_Point& p : points) {
        if (p.noise) continue;
        int tx = static_cast<int>(p.x);
        int ty = static_cast<int>(p.y);
        if (ty < 0 || ty >= height || tx < 0 || tx >= width) continue;
        data[ty][tx] = 100;  // 初始点值为100
        q.push({{tx, ty}, 100});
    }

    while (!q.empty()) {
        int_Point p = q.front().first;
        double value = q.front().second;
        q.pop();

        for (int i = 0; i < 4; ++i) {
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];
            if (nx < 0 || ny >= height || ny < 0 || nx >= width) {
                continue;
                // cout<<"out of range\n";
            }
            double next_value = value - 1 * decrease;
            if (data[ny][nx] < next_value) {
                data[ny][nx] = next_value;
                q.push({{nx, ny}, next_value});
            }
        }
        for (int i = 0; i < 4; i++) {
            int nx = p.x + dx2[i];
            int ny = p.y + dy2[i];
            if (nx < 0 || ny >= height || ny < 0 || nx >= width) {
                continue;
                // cout<<"out of range\n";
            }
            double next_value = value - 1.414 * decrease;
            if (data[ny][nx] < next_value) {
                data[ny][nx] = next_value; 
                q.push({{nx, ny}, next_value});
            }
        }
    }
}
//The obscured point filter
class obscured_point_filter{
    public:
        int width, height;
        struct point{
            int x;
            int y;
            double angle, distance;
            int angle_sort_id, distance_sort_id;
            int cnt;
        };
        class segment_tree{
            public:
                class segment_tree_node{
                    public:
                        segment_tree_node* left_son, *right_son;
                        int cnt, lazy_tag;
                        segment_tree_node(){
                            left_son = right_son = NULL;
                            cnt = 0;
                            lazy_tag = 0;
                        }
                };
                class insert_order{
                    public:
                        int x, y;
                        double angle;
                        insert_order(int x, int y, double angle){
                            this->x = x;
                            this->y = y;
                            this->angle = angle;
                        }
                        bool operator < (const insert_order& other)const{
                            return angle < other.angle;
                        }
                };
                void build(segment_tree_node* cur, int l, int r){
                    if(l == r)return;
                    int mid = (l + r) >> 1;
                    if(cur->left_son == NULL) cur->left_son = new segment_tree_node();
                    if(cur->right_son == NULL)cur->right_son = new segment_tree_node();
                    build(cur->left_son, l, mid);
                    build(cur->right_son, mid + 1, r);
                }
                void kill(segment_tree_node* cur){
                    if(cur->left_son != NULL)kill(cur->left_son);
                    if(cur->right_son != NULL)kill(cur->right_son);
                    delete cur;
                }
                segment_tree_node* root;
                segment_tree(int num_of_points){
                    root = new segment_tree_node();
                    build(root, 0, num_of_points - 1);
                }
                segment_tree() : root(NULL){}
                ~segment_tree(){
                    kill(root);
                }
                void pushdown(segment_tree_node* cur, int l, int r){
                    if(cur->lazy_tag == 0)return;
                    if(l == r)return;
                    int mid = (l + r) >> 1;
                    cur->left_son->lazy_tag += cur->lazy_tag;
                    cur->right_son->lazy_tag += cur->lazy_tag;
                    cur->left_son->cnt += cur->lazy_tag * (mid - l + 1);
                    cur->right_son->cnt += cur->lazy_tag * (r - mid);
                    cur->lazy_tag = 0;
                }
                void add(segment_tree_node* root, int l, int r, int x, int y){
                    if(x > y)return;
                    if(x <= l && r <= y){
                        root->cnt+=r-l+1;
                        root->lazy_tag++;
                        return;
                    }
                    pushdown(root, l, r);
                    int mid = (l + r) >> 1;
                    if(x <= mid)add(root->left_son, l, mid, x, y);
                    if(y > mid)add(root->right_son, mid + 1, r, x, y);
                    root->cnt = 0;
                    if(root->left_son != NULL)root->cnt += root->left_son->cnt;
                    if(root->right_son != NULL)root->cnt += root->right_son->cnt;
                }
                int query(segment_tree_node* root, int l, int r, int x, int y){
                    if(root == NULL)return 0;
                    if(x <= l && r <= y)return root->cnt;
                    pushdown(root, l, r);
                    int mid = (l + r) >> 1;
                    int res = 0;
                    if(x <= mid)res += query(root->left_son, l, mid, x, y);
                    if(y > mid)res += query(root->right_son, mid + 1, r, x, y);
                    return res;
                }
        };
        segment_tree* tree;
        vector<point> points;
        obscured_point_filter(int width, int height, vector<dbscan_Point>& obstacles){
            this->width = width;
            this->height = height;
            for(int i = 0; i < width; i++){
                for(int j = 0; j < height; j++){
                    point p;
                    p.x = i;
                    p.y = j;
                    p.angle = atan2(j - height / 2, i - width / 2);
                    p.distance = sqrt((j - height / 2) * (j - height / 2) + (i - width / 2) * (i - width / 2));
                    points.push_back(p);
                }
            }
            sort(points.begin(), points.end(), [&](point a, point b){
                return a.angle < b.angle;
            });
            vector<double> angle;
            int angle_cnt = 0;
            for(int i = 0; i < points.size(); i++){
                points[i].angle_sort_id = angle_cnt;
                if(i + 1 == points.size() || points[i + 1].angle != points[i].angle)angle_cnt++, angle.push_back(points[i].angle);
            }
            sort(points.begin(), points.end(), [&](point a, point b){
                return a.distance < b.distance;
            });
            vector<double> distance;
            int distance_cnt = 0;
            for(int i = 0; i < points.size(); i++){
                points[i].distance_sort_id = distance_cnt;
                if(i + 1 == points.size() || points[i + 1].distance != points[i].distance)distance_cnt++, distance.push_back(points[i].distance);
            }
            sort(points.begin(), points.end(), [&](point a, point b){
                return a.x < b.x || (a.x == b.x && a.y < b.y);
            });
            // ROS_INFO("Distance_cnt: %d", distance_cnt);
            // ROS_INFO("Angle_cnt: %d", angle_cnt);
            tree=new segment_tree(angle_cnt);
            // ROS_INFO("build tree success!");
            vector<vector<pair<int,int> > > angle_add(distance_cnt);
            // ROS_INFO("angle_add success!");
            for(int i = 0; i < obstacles.size(); i++){
                if(obstacles[i].noise)continue;
                int x = round(obstacles[i].x);
                int y = round(obstacles[i].y);
                double min_angle = min(atan2(y + 0.5 - height / 2, x + 0.5 - width / 2), min(atan2(y + 0.5 - height / 2, x - 0.5 - width / 2), min(atan2(y - 0.5 - height / 2, x + 0.5 - width / 2), atan2(y - 0.5 - height / 2, x - 0.5 - width / 2))));
                double max_angle = max(atan2(y + 0.5 - height / 2, x + 0.5 - width / 2), max(atan2(y + 0.5 - height / 2, x - 0.5 - width / 2), max(atan2(y - 0.5 - height / 2, x + 0.5 - width / 2), atan2(y - 0.5 - height / 2, x - 0.5 - width / 2))));
                // ROS_INFO("x: %d, y: %d", x, y);
                // ROS_INFO("min_angle: %lf, max_angle: %lf", min_angle, max_angle);
                if(max_angle - min_angle > M_PI){
                    auto low = lower_bound(angle.begin(), angle.end(), -M_PI), up = upper_bound(angle.begin(), angle.end(), min_angle);
                    int low_index = low - angle.begin(), up_index = up - angle.begin() - 1;
                    angle_add[points[x*width+y].distance_sort_id].push_back({low_index, up_index});
                    // ROS_INFO("low_index: %d, up_index: %d, Angle_cnt %d", low_index, up_index, angle_cnt);
                    low = lower_bound(angle.begin(), angle.end(), max_angle), up = upper_bound(angle.begin(), angle.end(), M_PI);
                    low_index = low - angle.begin(), up_index = up - angle.begin() - 1;
                    angle_add[points[x*width+y].distance_sort_id].push_back({low_index, up_index});
                    // ROS_INFO("low_index: %d, up_index: %d, Angle_cnt %d", low_index, up_index, angle_cnt);
                }
                else{
                    auto low = lower_bound(angle.begin(), angle.end(), min_angle), up = upper_bound(angle.begin(), angle.end(), max_angle);
                    int low_index = low - angle.begin(), up_index = up - angle.begin() - 1;
                    angle_add[points[x*width+y].distance_sort_id].push_back({low_index, up_index});
                    // ROS_INFO("low_index: %d, up_index: %d, Angle_cnt %d", low_index, up_index, angle_cnt);
                }
            }
            sort(points.begin(), points.end(), [&](point a, point b){
                return a.distance < b.distance;
            });
            int index = 0;
            for(int i = 0; i < distance_cnt; i++){
                for(int j = 0; j < angle_add[i].size(); j++){
                    tree->add(tree->root, 0, angle_cnt - 1, angle_add[i][j].first, angle_add[i][j].second);
                    // ROS_INFO("low_index: %d, up_index: %d", angle_add[i][j].first, angle_add[i][j].second);
                }
                while(index < points.size() && points[index].distance_sort_id == i){
                    points[index].cnt = tree->query(tree->root, 0, angle_cnt - 1, points[index].angle_sort_id, points[index].angle_sort_id);
                    index++;
                }
            }
        }
        vector<pair<int,int> > get_obscured_points(){
            vector<pair<int,int> > res;
            for(int i = 0; i < points.size(); i++)if(points[i].cnt>1){
                res.push_back({points[i].x, points[i].y});
                // ROS_INFO("x: %d, y: %d", points[i].x, points[i].y);
            }
            return res;
        }
        ~obscured_point_filter(){
            delete tree;
        }
        
};

ros::Publisher grid_pub;
double robot_pos_x = 0.0, robot_pos_y = 0.0, robot_pos_z = 0.0, 
        robot_quat_x = 0.0, robot_quat_y = 0.0, robot_quat_z = 0.0, robot_quat_w = 0.0;
double map_pos_x = 0.0, map_pos_y = 0.0, map_pos_z = 0.0, 
        map_quat_x = 0.0, map_quat_y = 0.0, map_quat_z = 0.0, map_quat_w = 0.0;

// following get by ros param
// int obstacle_enlargement=4;
// int MAXN=200;Z
double epsilon = 0.2; //聚类的阈值
int minPts = 7; //聚类的最小点数
int dfs_decrease = 5;
int dfs_threshold = 50;
string frame_name = "gimbal_frame";
string parent_frame, child_frame;

int max_radar_x = -1e8, max_radar_y = -1e8, min_radar_x = 1e8, min_radar_y = 1e8;
// extern tf2_ros::TransformListener tfListener(tfBuffer);
geometry_msgs::TransformStamped transformStamped;
pcl::PointCloud<pcl::PointXYZ>::Ptr scan_record(new pcl::PointCloud<pcl::PointXYZ>);
bool get_scan;
void msgs_to_grid(const sensor_msgs::PointCloud2ConstPtr& msg)
{   
    pcl::fromROSMsg(*msg, *scan_record);
    // ROS_INFO("Get map!");
    // printf("get_scan");
    // 声明grid变量
    nav_msgs::OccupancyGrid grid;

    // 设置占用网格的基本属性
    grid.header.frame_id = frame_name;// 调试雷达时改为"odom'->"laser"
    grid.info.resolution = 0.05;  // 网格的分辨率为0.05米
    grid.info.width = MAXN;  // 网格的宽度为500个单元 uint32
    grid.info.height = MAXN;  // 网格的高度为500个单元 uint32

    // 初始化占用网格的数据
    grid.data.resize(grid.info.width * grid.info.height, 0);
    // 转换数据类型 
    float x_origin = -static_cast<float>(grid.info.width * grid.info.resolution / 2.0);
    float y_origin = -static_cast<float>(grid.info.height * grid.info.resolution / 2.0);
    grid.info.origin.position.x = x_origin;// ladar
    grid.info.origin.position.y = y_origin;// ladar
    grid.info.origin.position.z = 0.0;
    // 创建一个二维数组
    vector<vector<double> > grid_vector(grid.info.height, vector<double>(grid.info.width, 0));
    // 创建储存雷达数据的二维数组
    int data_size = scan_record->points.size();
    // 将激光扫描数据转换为占用网格

    //dbscan
    vector<dbscan_Point> dbscan_data;
    for(size_t i = 0; i < scan_record->points.size(); ++i){
        dbscan_Point p;
        // 计算激光点的坐标(极坐标系转换为笛卡尔坐标系)
        p.x= scan_record->points[i].x;
        p.y= scan_record->points[i].y;
        p.z= scan_record->points[i].z;
        if(isnan(p.x)||isnan(p.y)||isnan(p.z))continue;
        if(round((p.x - x_origin) / grid.info.resolution)<0 || round((p.x - x_origin) / grid.info.resolution)>=grid.info.width || round((p.y - y_origin) / grid.info.resolution)<0 || round((p.y - y_origin) / grid.info.resolution)>=grid.info.height)continue;
        // cout<<p.x<<' '<<p.y<<"\n";
        dbscan_data.push_back(p);
    }
    int cnt=0;
    dbscan(dbscan_data, epsilon, minPts, cnt);

        // 将笛卡尔坐标转换为占用网格的索引 原点在网格的中心
    for(size_t i = 0; i < dbscan_data.size(); ++i){
        dbscan_data[i].x =(dbscan_data[i].x - x_origin) / grid.info.resolution;
        dbscan_data[i].y =(dbscan_data[i].y - y_origin) / grid.info.resolution;
        if(!dbscan_data[i].noise){
            int x = round(dbscan_data[i].x);
            int y = round(dbscan_data[i].y);
            grid_vector[y][x] = 100;
        }
    }
        
    //膨胀
    bfs(dbscan_data,grid_vector,grid.info.width,grid.info.height, dfs_decrease);

    for (int i = 0; i < grid.info.height; ++i) {
        for (int j = 0; j < grid.info.width; ++j) {
            int index = i * grid.info.width + j;
            //阈值设置
            grid.data[index] = max(0, int(grid_vector[i][j]));
        }
    }
    auto start_ = chrono::high_resolution_clock::now();
    obscured_point_filter opf(grid.info.width, grid.info.height, dbscan_data);
    vector<pair<int,int> > obscured_points = opf.get_obscured_points();
    for(int i = 0; i < obscured_points.size(); i++){
        int x = obscured_points[i].first;
        int y = obscured_points[i].second;
        int index = y * grid.info.width + x;
        if(grid.data[index]!=100)grid.data[index] = -1;
    }
    
    auto end_ = chrono::high_resolution_clock::now();
    chrono::duration<double> diff = end_-start_;

    grid_pub.publish(grid);
    cout << "Time difference: " << diff.count() << " s\n";
    
}


int main(int argc, char **argv)
{
            printf("start");
    string node_name = "dbscan_bfs_3D";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh; // 句柄 创建了一个NodeHandle对象 提供了c++与ROS系统交互的接口
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    if (!ros::param::get("/" + node_name + "/epsilon", epsilon)) {
        ROS_ERROR("Failed to get param 'epsilon'");
    }
    else{
        ROS_INFO("Got param 'epsilon': %lf", epsilon);
    }
    if (!ros::param::get("/" + node_name + "/minPts", minPts)) {
        ROS_ERROR("Failed to get param 'minPts'");
    }
    else{
        ROS_INFO("Got param 'minPts': %d", minPts);
    }
    if (!ros::param::get("/" + node_name + "/dfs_decrease", dfs_decrease)) {
        ROS_ERROR("Failed to get param 'dfs_decrease'");
    }
    else{
        ROS_INFO("Got param 'dfs_decrease': %d", dfs_decrease);
    }
    if (!ros::param::get("/" + node_name + "/dfs_threshold", dfs_threshold)) {
        ROS_ERROR("Failed to get param 'dfs_threshold'");
    }
    else{
        ROS_INFO("Got param 'dfs_threshold': %d", dfs_threshold);
    }
    if (!ros::param::get("/" + node_name + "/child_frame", child_frame)) {
        ROS_ERROR("Failed to get param 'child_frame'");
    }
    else{
        ROS_INFO("Got param 'child_frame': %s", child_frame.c_str());
    }
    if (!ros::param::get("/" + node_name + "/parent_frame", parent_frame)) {
        ROS_ERROR("Failed to get param 'parent_frame'");
    }
    else{
        ROS_INFO("Got param 'parent_frame': %s", parent_frame.c_str());
    }
    frame_name=child_frame;
    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("grid", 1); // (Topic Name, Queue Size)
    
    ros::Subscriber sub_1 = nh.subscribe("/test_scan", 1, msgs_to_grid); // 订阅sensor_msgs/LaserScan 并转换(Topic Name, Queue Size, Callback Function)
    // ros::Subscriber sub_2 = nh.subscribe("odom", 10, &odom_callback);//订阅mav_msgs/Odometr
    // ros::Timer timer = nh.createTimer(ros::Duration(0.1), publish_transform);  // 每隔0.1秒发布一次坐标变换
    ros::spin();

    return 0;
}
