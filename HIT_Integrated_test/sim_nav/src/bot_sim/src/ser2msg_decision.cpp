#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <vector>

const double PI = 3.14159265358979323846;

serial::Serial ser;
const int write_length = 15;
const int read_length = 19;

geometry_msgs::TransformStamped transformRotbaseToVirtual;
geometry_msgs::TransformStamped transformGimbalToRotbase;
geometry_msgs::TransformStamped transformMapToGimbal;
geometry_msgs::TransformStamped loc;
bool status = 0;

union FloatToByte{
    float f;
    uint8_t bytes[sizeof(float)];
};

union ByteToByte{
    uint8_t f;
    uint8_t bytes[sizeof(float)];
};



class Message {
public:
    static const uint8_t SOF = 0x91;
    static const uint8_t eof = 0xFE;
    // float imu_change_threshold = 0.2;
    // float relative_change_threshold = 15;
    // float past_imu_angle;
    // float past_relative_angle;
    float imu_angle = 0;
    float relative_angle = 0; float goal_x = 0; float goal_y = 0;
    int goal_type = 12;
    bool receive_message = 0;
    std::vector<uint8_t> buffer;

    bool readFromBuffer() {
        if (this->buffer.size() < read_length) {
            // ROS_INFO("Not enough data available from the serial port, current buffer size: %d", buffer.size());
            return false; // Not enough data, 
        }
        else if (this->buffer[0] == SOF && this->buffer[read_length-1] == eof) {
            memcpy(&this->relative_angle, &this->buffer[1], sizeof(float));
            memcpy(&this->imu_angle, &this->buffer[5], sizeof(float));
            unsigned char temp;
            memcpy(&temp, &this->buffer[9], sizeof(char));
            // ROS_INFO("goal_type: %d", (unsigned int)temp);
            this->goal_type = (unsigned int)temp;
            memcpy(&this->goal_x, &this->buffer[10], sizeof(float));
            memcpy(&this->goal_y, &this->buffer[14], sizeof(float));
            this->buffer.erase(this->buffer.begin(), this->buffer.begin()+read_length);
            this->receive_message = 1;
            // ROS_INFO("Read from buffer");
            return true;
        }
        else {
            this->buffer.erase(this->buffer.begin());
            return true; // Format dismatch, drop the first byte and try again 
        }
    }
    void printData() {
        ROS_INFO_STREAM("imu_angle: " << this->imu_angle);
        ROS_INFO_STREAM("relative_angle: " << this->relative_angle);
    }
};

geometry_msgs::Twist cmd_vel;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel = *msg;
    // printf("msg_received");
    // std::cout<<cmd_vel.linear.x<<' '<<cmd_vel.linear.y<<std::endl;
}
void StatusCallback(const std_msgs::Bool::ConstPtr &msg)
{
    // cmd_vel = *msg;
    status = 1;
    // printf("msg_received");
    // std::cout<<cmd_vel.linear.x<<' '<<cmd_vel.linear.y<<std::endl;
}
// std::pair<double, double> getGoalType[]={
// {8.215254783630371, 8.674027442932129},    //0
// {8.011299133300781, 25.099763870239258},   //1
// {5.136077880859375, 13.348814010620117},   //2
// {10.739795684814453, 20.710840225219727},  //3
// {2.661466598510742, 17.89205551147461},    //4
// {13.056159973144531,  15.860123634338379}, //5
// {4.178987979888916, 20.687192916870117},   //6
// {4.229683876037598, 11.059250831604004},   //7
// {11.635642051696777, 22.893033981323242},  //8
// {13.664833068847656, 11.683853149414062},  //9
// {14.618308067321777, 5.061775207519531},   //10
// {13.056159973144531,  15.860123634338379}  //11
// };
// double angles[2]={0.0, -35.0/180.0*3.14159265};


// The following are for testing at Jul29
// std::pair<double, double> getGoalType[]={
// {0.37, 0.24},    //0
// {8.011299133300781, 25.099763870239258},   //1
// {-1.04, -0.02},   //2-->10
// {10.739795684814453, 20.710840225219727},  //3
// {2.661466598510742, 17.89205551147461},    //4
// {0.6, -6.3}, //5-.6
// {0.55, -3.3},   //6-->2
// {4.229683876037598, 11.059250831604004},   //7
// {11.635642051696777, 22.893033981323242},  //8
// {13.664833068847656, 11.683853149414062},  //9
// {2.154, -0.227},   //10
// {13.056159973144531,  15.860123634338379}  //11
// };
std::vector<std::pair<double, double> > getGoalType[]={
{{-0.15, 1.1}, {0.47, -1.13}},    //0
{{8.011299133300781, 25.099763870239258}},   //1
{{2.15, -2.05}},//2
{{10.739795684814453, 20.710840225219727}},  //3
{{2.661466598510742, 17.89205551147461}},    //4
{{13.664833068847656, 11.683853149414062}},  //5
{{0.7, -1.6}}, //6
{{4.229683876037598, 11.059250831604004}},   //7
{{3.3, -5.8}},   //8
{{11.635642051696777, 22.893033981323242}},  //9
{{3.21, 1.26}},   //10
{{13.056159973144531,  15.860123634338379}}  //11
};
double angles[2]={0.0, 0.0};


int main(int argc, char** argv)
{
    std::string node_name = "ser2msg_decision";
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;

    std::string serial_port;
    if (!nh.getParam("/"+node_name+"/serial_port", serial_port))
    {
        ROS_ERROR("Failed to retrieve parameter 'serial_port'");
        return -1;
    }
    float delta_time;
    if (!nh.getParam("/"+node_name+"/delta_time", delta_time))
    {
        ROS_ERROR("Failed to retrieve parameter 'delta_time'");
        return -1;
    }
    std::string virtual_frame;
    if (!nh.getParam("/"+node_name+"/virtual_frame", virtual_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'virtual_frame'");
        return -1;
    }
    std::string rotbase_frame;
    if (!nh.getParam("/"+node_name+"/rotbase_frame", rotbase_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'rotbase_frame'");
        return -1;
    }
    std::string gimbal_frame;
    if (!nh.getParam("/"+node_name+"/gimbal_frame", gimbal_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'gimbal_frame'");
        return -1;
    }
    std::string vel_topic;
    if (!nh.getParam("/"+node_name+"/vel_topic",vel_topic)){
    	ROS_ERROR("Failed to get param: %s", vel_topic.c_str());
    }
    std::string _3DLidar_frame;
    if (!nh.getParam("/"+node_name+"/_3DLidar_frame", _3DLidar_frame))
    {
        ROS_ERROR("Failed to retrieve parameter '_3DLidar_frame'");
        return -1;
    }

    // message.setTransform(virtual_frame, rotbase_frame);

    try
    {
        // Configure the serial port
        ser.setPort(serial_port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(100000);
        ser.setTimeout(to);
        ser.setBytesize(serial::eightbits);
        // ser.setStopbits(serial::stopbits_one);
        ser.setParity(serial::parity_none);
        ser.setFlowcontrol(serial::flowcontrol_none);

        // Open the serial port
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open the serial port");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial port initialized");
    }
    else
    {
        return -1;
    }

    ros::Subscriber sub = nh.subscribe(vel_topic, 1000, cmdVelCallback);
    ros::Subscriber sub_status = nh.subscribe("/dstar_status", 1000, StatusCallback);
    ros::Publisher clicked_point_pub=nh.advertise<geometry_msgs::PointStamped>("clicked_point",1);

    tf2_ros::TransformBroadcaster tfb;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    tf2::Quaternion q1;
    transformRotbaseToVirtual.header.frame_id = rotbase_frame;
    transformRotbaseToVirtual.child_frame_id = virtual_frame;
    transformRotbaseToVirtual.transform.translation.x = 0.0;
    transformRotbaseToVirtual.transform.translation.y = 0.0;
    transformRotbaseToVirtual.transform.translation.z = -0.1;

    
    tf2::Quaternion q2;
    transformGimbalToRotbase.header.frame_id = gimbal_frame;
    transformGimbalToRotbase.child_frame_id = rotbase_frame;
    transformGimbalToRotbase.transform.translation.x = 0.0;
    transformGimbalToRotbase.transform.translation.y = 0.0;
    transformGimbalToRotbase.transform.translation.z = -0.3;

    Message message;
    uint8_t byte;
    uint8_t buffer_send[read_length];
    buffer_send[0] = 0x4A; // SOF
    ros::Rate rate = ros::Rate(1/delta_time);

    tf2::Transform gimbalframe;
    tf2::Transform rotbaseframe;
    tf2::Transform virtualframe;
    tf2::Transform location;

    loc.header.frame_id = "map";
    loc.child_frame_id = virtual_frame;

    int last_goal = -1, current_goal = -1, current_index = 0, cnt = 0;
    bool first_arrive_flag = 0, cnt_flag = 0;
    
    while(ros::ok()){

        // Read data from the serial port
        // ROS_INFO("Reading data from the serial port");

        size_t bytes_available = ser.available();

        while (ser.available() && ros::ok())
        {
            ser.read(&byte,1);
            message.buffer.push_back(byte);
            // ROS_INFO("%d", message.buffer.size());
        }
        while (message.readFromBuffer() && ros::ok());
            // message.printData();
        //Calculate virtual frame
        {
            q1.setRPY(0,0,-message.imu_angle);
            transformRotbaseToVirtual.transform.rotation.x = q1.x();
            transformRotbaseToVirtual.transform.rotation.y = q1.y();
            transformRotbaseToVirtual.transform.rotation.z = q1.z();
            transformRotbaseToVirtual.transform.rotation.w = q1.w();
            transformRotbaseToVirtual.header.stamp = ros::Time::now();
            // tfb.sendTransform(transformRotbaseToVirtual);

            q2.setRPY(0,0,-message.relative_angle);
            transformGimbalToRotbase.transform.rotation.x = q2.x();
            transformGimbalToRotbase.transform.rotation.y = q2.y();
            transformGimbalToRotbase.transform.rotation.z = q2.z();
            transformGimbalToRotbase.transform.rotation.w = q2.w();
            transformGimbalToRotbase.header.stamp = ros::Time::now();
            // tfb.sendTransform(transformGimbalToRotbase);
            

            try{
                transformMapToGimbal = tfBuffer.lookupTransform("map", "gimbal_frame",ros::Time(0),ros::Duration(5.0));
            }
            catch(tf2::TransformException &ex){
                ROS_WARN("%s",ex.what());
            }
            tf2::fromMsg(transformMapToGimbal.transform, gimbalframe);
            tf2::fromMsg(transformGimbalToRotbase.transform,rotbaseframe);
            tf2::fromMsg(transformRotbaseToVirtual.transform,virtualframe);
            // location = gimbalframe;
            // // * rotbaseframe * virtualframe;
            location = gimbalframe * rotbaseframe * virtualframe;
            loc.transform = tf2::toMsg(location);
            loc.header.stamp = ros::Time::now();
            tfb.sendTransform(loc);
            
        }
        // ROS_INFO("continue 1");
        //send goal
        {
            ROS_INFO("goal_type: %d",message.goal_type);
            if(message.goal_type != current_goal){
                last_goal = current_goal;
                current_goal = message.goal_type;
                current_index = 0;
                cnt = cnt_flag = 0;
                first_arrive_flag = 0;
            }
            // ROS_INFO("continue 1 1");
            if(message.goal_type != 0x0C){
                geometry_msgs::PointStamped clicked_point;
                clicked_point.header.frame_id="map";
                clicked_point.header.stamp=ros::Time::now();
                // ROS_INFO("continue 1 2");
                if(message.goal_type != 0xF0){
                    if(status){
                        cnt_flag = 1;
                    } 
                    if(cnt_flag){
                        cnt++;
                    }
                    if(cnt == 8 / delta_time){
                        cnt = cnt_flag = 0;
                        current_index = (current_index + 1) %  getGoalType[message.goal_type].size();
                    }
                    clicked_point.point.x=getGoalType[message.goal_type][current_index].first;
                    clicked_point.point.y=getGoalType[message.goal_type][current_index].second;
                    clicked_point.point.z=0;
                }
                else{
                    clicked_point.point.x=message.goal_x;
                    clicked_point.point.y=message.goal_y;
                    clicked_point.point.z=0;
                }
                clicked_point_pub.publish(clicked_point);
            }
        }
        // ROS_INFO("continue 2");
        // Write data to serial port
        // Linear velocities x
        {   
            FloatToByte linear_x;
            linear_x.f = cmd_vel.linear.x;
            std::copy(std::begin(linear_x.bytes),std::end(linear_x.bytes),&buffer_send[1]);
        }
        // Linear velocities y
        {
            FloatToByte linear_y;
            linear_y.f = -cmd_vel.linear.y;
            std::copy(std::begin(linear_y.bytes),std::end(linear_y.bytes),&buffer_send[5]);
        }
        // ROS_INFO("continue 3");
        // Omega angle w
        {
            FloatToByte omega;
            int index = 0;
            if(current_goal == 2 || last_goal == 2){
                index = 1;
            }
            auto tf_rot_from_map_to_virtual = loc;
            tf_rot_from_map_to_virtual.transform.translation.x = 0;
            tf_rot_from_map_to_virtual.transform.translation.y = 0;
            tf_rot_from_map_to_virtual.transform.translation.z = 0;
            geometry_msgs::PointStamped source_point;
            source_point.header.frame_id = "virtual_frame";
            source_point.header.stamp = ros::Time(0);
            source_point.point.x = cos(angles[index]);
            source_point.point.y = sin(angles[index]);
            ROS_INFO("x: %lf y: %lf angle: %lf index: %d", source_point.point.x, source_point.point.y, angles[index], index);
            source_point.point.z = 0;
            geometry_msgs::PointStamped target_point;
            tf2::doTransform(source_point, target_point, tf_rot_from_map_to_virtual);
            double rx = target_point.point.x;
            double ry = target_point.point.y;
            double angle = atan2(ry,rx);
            omega.f = -angle;
            std::copy(std::begin(omega.bytes),std::end(omega.bytes),&buffer_send[9]);
        }
        //received message
        {
            ByteToByte received_message;
            received_message.f = message.receive_message;
            std::copy(std::begin(received_message.bytes),std::end(received_message.bytes),&buffer_send[13]);
            message.receive_message = 0;
        }
        // ROS_INFO("continue 4");
        //whether arrived
        {
            ByteToByte arrived;//0 move, 1 arrive
            arrived.f = first_arrive_flag | (message.goal_type == 12);
            first_arrive_flag |= status;
            status = 0;
            std::copy(std::begin(arrived.bytes),std::end(arrived.bytes),&buffer_send[14]);
        }
        // Write to the serial port
        size_t bytes_written = ser.write(buffer_send,write_length);
        if (bytes_written < write_length){
            ROS_ERROR("Failed to write all bytes to the serial port");
        }
        ROS_INFO("relative angle: %f imu_angle: %f", message.relative_angle, message.imu_angle);
        // ROS_INFO("%f %f %f", linear_x.f, linear_y.f, omega.f);
        ros::spinOnce(); 
        rate.sleep();
    }
    ros::spin();
    return 0;
}