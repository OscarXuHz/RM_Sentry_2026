#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>

class CommandTester
{
public:
    CommandTester() : nh_(""), nav_frequency_(50), nav_enabled_(false), 
                      nav_vx_(0.0f), nav_vy_(0.0f), nav_z_angle_(0.0f)
    {
        // 创建发布者
        pub_motion_ = nh_.advertise<std_msgs::UInt8>("motion", 1);
        pub_recover_ = nh_.advertise<std_msgs::UInt8>("recover", 1);
        pub_bullet_up_ = nh_.advertise<std_msgs::UInt8>("bullet_up", 1);
        pub_bullet_num_ = nh_.advertise<std_msgs::UInt8>("bullet_num", 1);
        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        pub_nav_received_ = nh_.advertise<std_msgs::UInt8>("nav_received", 1);
        pub_dstar_status_ = nh_.advertise<std_msgs::Bool>("/dstar_status", 1);
        
        ROS_INFO("Command Tester initialized");
        ROS_INFO("Publishers created for:");
        ROS_INFO("  - motion (std_msgs::UInt8)");
        ROS_INFO("  - recover (std_msgs::UInt8)");
        ROS_INFO("  - bullet_up (std_msgs::UInt8)");
        ROS_INFO("  - bullet_num (std_msgs::UInt8)");
        ROS_INFO("  - cmd_vel (geometry_msgs::Twist) - variable frequency publishing");
        ROS_INFO("  - nav_received (std_msgs::UInt8)");
        ROS_INFO("  - /dstar_status (std_msgs::Bool)");
        
        // 启动导航定时发送线程
        nav_publish_thread_ = std::thread(&CommandTester::navigationPublishLoop, this);
    }
    
    ~CommandTester()
    {
        if (nav_publish_thread_.joinable())
        {
            nav_publish_thread_.join();
        }
    }
    
    void run()
    {
        std::string command;
        int value;
        
        while (ros::ok())
        {
            printMenu();
            
            std::cout << "\nEnter command (motion/recover/bullet_up/bullet_num/navigation/nav_received/dstar/quit): ";
            std::getline(std::cin, command);
            
            if (command == "quit" || command == "q")
            {
                ROS_INFO("Exiting command tester...");
                break;
            }
            
            if (command == "motion")
            {
                std::cout << "Enter motion value (0-255): ";
                if (std::cin >> value)
                {
                    std::cin.ignore(); // Clear newline from input buffer
                    
                    if (value >= 0 && value <= 255)
                    {
                        std_msgs::UInt8 msg;
                        msg.data = static_cast<uint8_t>(value);
                        pub_motion_.publish(msg);
                        ROS_INFO("Published motion: %d", value);
                    }
                    else
                    {
                        ROS_WARN("Invalid motion value. Please enter a value between 0-255");
                    }
                }
                else
                {
                    std::cin.clear();
                    std::cin.ignore(10000, '\n');
                    ROS_WARN("Invalid input. Please enter a number");
                }
            }
            else if (command == "recover")
            {
                std::cout << "Enter recover value (0-255): ";
                if (std::cin >> value)
                {
                    std::cin.ignore(); // Clear newline from input buffer
                    
                    if (value >= 0 && value <= 255)
                    {
                        std_msgs::UInt8 msg;
                        msg.data = static_cast<uint8_t>(value);
                        pub_recover_.publish(msg);
                        ROS_INFO("Published recover: %d", value);
                    }
                    else
                    {
                        ROS_WARN("Invalid recover value. Please enter a value between 0-255");
                    }
                }
                else
                {
                    std::cin.clear();
                    std::cin.ignore(10000, '\n');
                    ROS_WARN("Invalid input. Please enter a number");
                }
            }
            else if (command == "bullet_up")
            {
                std::cout << "Enter bullet_up value (0-255): ";
                if (std::cin >> value)
                {
                    std::cin.ignore(); // Clear newline from input buffer
                    
                    if (value >= 0 && value <= 255)
                    {
                        std_msgs::UInt8 msg;
                        msg.data = static_cast<uint8_t>(value);
                        pub_bullet_up_.publish(msg);
                        ROS_INFO("Published bullet_up: %d", value);
                    }
                    else
                    {
                        ROS_WARN("Invalid bullet_up value. Please enter a value between 0-255");
                    }
                }
                else
                {
                    std::cin.clear();
                    std::cin.ignore(10000, '\n');
                    ROS_WARN("Invalid input. Please enter a number");
                }
            }
            else if (command == "bullet_num")
            {
                std::cout << "Enter bullet_num value (0-255): ";
                if (std::cin >> value)
                {
                    std::cin.ignore(); // Clear newline from input buffer
                    
                    if (value >= 0 && value <= 255)
                    {
                        std_msgs::UInt8 msg;
                        msg.data = static_cast<uint8_t>(value);
                        pub_bullet_num_.publish(msg);
                        ROS_INFO("Published bullet_num: %d", value);
                    }
                    else
                    {
                        ROS_WARN("Invalid bullet_num value. Please enter a value between 0-255");
                    }
                }
                else
                {
                    std::cin.clear();
                    std::cin.ignore(10000, '\n');
                    ROS_WARN("Invalid input. Please enter a number");
                }
            }
            else if (command == "navigation")
            {
                float vx, vy, z_angle;
                
                std::cout << "Enter vx (velocity x, float): ";
                if (std::cin >> vx)
                {
                    std::cout << "Enter vy (velocity y, float): ";
                    if (std::cin >> vy)
                    {
                        std::cout << "Enter z_angle (angle, float): ";
                        if (std::cin >> z_angle)
                        {
                            std::cin.ignore(); // Clear newline from input buffer
                            
                            // 设置导航数据
                            {
                                std::lock_guard<std::mutex> lock(nav_mutex_);
                                nav_vx_ = vx;
                                nav_vy_ = vy;
                                nav_z_angle_ = z_angle;
                                nav_enabled_ = true;  // 启用导航发送
                            }
                            
                            ROS_INFO("Navigation (cmd_vel) set: vx=%.4f, vy=%.4f, z_angle=%.4f", vx, vy, z_angle);
                            ROS_INFO("   Publishing to /cmd_vel topic...");
                        }
                        else
                        {
                            std::cin.clear();
                            std::cin.ignore(10000, '\n');
                            ROS_WARN("Invalid input for z_angle. Please enter a number");
                        }
                    }
                    else
                    {
                        std::cin.clear();
                        std::cin.ignore(10000, '\n');
                        ROS_WARN("Invalid input for vy. Please enter a number");
                    }
                }
                else
                {
                    std::cin.clear();
                    std::cin.ignore(10000, '\n');
                    ROS_WARN("Invalid input for vx. Please enter a number");
                }
            }
            else if (command == "nav_received")
            {
                std::cout << "Enter nav_received value (0-255): ";
                if (std::cin >> value)
                {
                    std::cin.ignore(); // Clear newline from input buffer
                    
                    if (value >= 0 && value <= 255)
                    {
                        std_msgs::UInt8 msg;
                        msg.data = static_cast<uint8_t>(value);
                        pub_nav_received_.publish(msg);
                        ROS_INFO("Published nav_received: %d", value);
                    }
                    else
                    {
                        ROS_WARN("Invalid nav_received value. Please enter a value between 0-255");
                    }
                }
                else
                {
                    std::cin.clear();
                    std::cin.ignore(10000, '\n');
                    ROS_WARN("Invalid input. Please enter a number");
                }
            }
            else if (command == "dstar")
            {
                std::cout << "Enter dstar_status value (0=not arrived, 1=arrived): ";
                if (std::cin >> value)
                {
                    std::cin.ignore(); // Clear newline from input buffer
                    
                    if (value == 0 || value == 1)
                    {
                        std_msgs::Bool msg;
                        msg.data = (value != 0);
                        pub_dstar_status_.publish(msg);
                        ROS_INFO("Published /dstar_status: %s", msg.data ? "true (arrived)" : "false (not arrived)");
                    }
                    else
                    {
                        ROS_WARN("Invalid dstar_status value. Please enter 0 (not arrived) or 1 (arrived)");
                    }
                }
                else
                {
                    std::cin.clear();
                    std::cin.ignore(10000, '\n');
                    ROS_WARN("Invalid input. Please enter a number");
                }
            }
            else if (!command.empty())
            
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }
    
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_motion_;
    ros::Publisher pub_recover_;
    ros::Publisher pub_bullet_up_;
    ros::Publisher pub_bullet_num_;
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_nav_received_;
    ros::Publisher pub_dstar_status_;
    
    // 导航定时发送相关
    int nav_frequency_;                      // 发送频率 (Hz)
    bool nav_enabled_;                       // 是否启用导航发送
    float nav_vx_, nav_vy_, nav_z_angle_;   // 导航数据
    std::mutex nav_mutex_;                   // 线程同步
    std::thread nav_publish_thread_;         // 发送线程
    
    // 导航后台定时发送线程
    void navigationPublishLoop()
    {
        ros::Rate rate(nav_frequency_);
        
        ROS_INFO("Navigation publish thread started at %.0f Hz", (double)nav_frequency_);
        
        while (ros::ok())
        {
            if (nav_enabled_)
            {
                // 获取最新的导航数据
                float vx, vy, z_angle;
                {
                    std::lock_guard<std::mutex> lock(nav_mutex_);
                    vx = nav_vx_;
                    vy = nav_vy_;
                    z_angle = nav_z_angle_;
                }
                
                // 发布导航消息为Twist
                geometry_msgs::Twist msg;
                msg.linear.x = vx;
                msg.linear.y = vy;
                msg.angular.z = z_angle;
                pub_cmd_vel_.publish(msg);
                
                // 每 50 帧输出一次日志
                static int publish_count = 0;
                publish_count++;
                if (publish_count % 50 == 0)
                {
                    ROS_DEBUG("Twist published: vx=%.4f, vy=%.4f, z_angle=%.4f (count=%d)", 
                             vx, vy, z_angle, publish_count);
                }
            }
            
            rate.sleep();
        }
    }
    
    void printMenu()
    {
        std::cout << "\n========== Command Menu ==========" << std::endl;
        std::cout << "  motion     - Send motion command (0-255)" << std::endl;
        std::cout << "  recover    - Send recover command (0-255)" << std::endl;
        std::cout << "  bullet_up  - Send bullet_up command (0-255)" << std::endl;
        std::cout << "  bullet_num - Send bullet_num command (0-255)" << std::endl;
        std::cout << "  navigation - Set & start navigation (vx, vy, z_angle as floats)" << std::endl;
        std::cout << "              Publish to /cmd_vel topic" << std::endl;
        std::cout << "  nav_received - Send nav_received command (0-255)" << std::endl;
        std::cout << "  dstar      - Send /dstar_status command (0=not arrived, 1=arrived)" << std::endl;
        std::cout << "  quit       - Exit program" << std::endl;
        std::cout << "==================================" << std::endl;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "command_tester");
    
    try
    {
        CommandTester tester;
        tester.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Fatal error: %s", e.what());
        return 1;
    }
    
    return 0;
}
