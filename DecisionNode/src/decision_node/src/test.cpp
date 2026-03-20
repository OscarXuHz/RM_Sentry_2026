// continuous_forwarder.cpp
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "continuous_forwarder");
    ros::NodeHandle nh;
    
    ros::Publisher pub_game = nh.advertise<std_msgs::UInt8>("/referee/game_progress", 1);
    ros::Publisher pub_hp = nh.advertise<std_msgs::UInt16>("/referee/remain_hp", 1);
    ros::Publisher pub_bullet = nh.advertise<std_msgs::UInt16>("/referee/bullet_remain", 1);
    ros::Publisher pub_friendly = nh.advertise<std_msgs::Int32>("/referee/friendly_score", 1);
    ros::Publisher pub_enemy = nh.advertise<std_msgs::Int32>("/referee/enemy_score", 1);
    ros::Publisher pub_arrived = nh.advertise<std_msgs::Bool>("/dstar_status", 1);
    ros::Publisher pub_occupy = nh.advertise<std_msgs::UInt8>("/referee/occupy_status", 1);
    
    ros::Duration(0.2).sleep();
    
    std::cout << "\n=== 持续数据转发器 ===\n";
    std::cout << "格式: <话题> <数值>\n";
    std::cout << "话题: 1游戏进度(0-2) 2HP(0-400) 3子弹 4友方分 5敌方分 6到达(0/1) 7占领状态(0-3)\n";
    std::cout << "占领状态: 0=未占 1=友方占 2=敌方占 3=双方占\n";
    std::cout << "输入 send 发送所有数据，输入 q 退出\n\n";
    
    int game_progress = 0;
    int hp = 400;
    int bullet = 999;
    int friendly_score = 0;
    int enemy_score = 0;
    bool arrived = false;
    int occupy_status = 0;
    
    while (ros::ok())
    {
        std::string input;
        std::cout << "> ";
        std::cin >> input;
        
        if (input == "q" || input == "quit")
        {
            std::cout << "退出程序\n";
            break;
        }
        else if (input == "send")
        {
            std_msgs::UInt8 uint8_msg;
            std_msgs::UInt16 uint16_msg;
            std_msgs::Int32 int_msg;
            std_msgs::Bool bool_msg;
            
            uint8_msg.data = game_progress;
            pub_game.publish(uint8_msg);
            
            uint16_msg.data = hp;
            pub_hp.publish(uint16_msg);
            
            uint16_msg.data = bullet;
            pub_bullet.publish(uint16_msg);
            
            int_msg.data = friendly_score;
            pub_friendly.publish(int_msg);
            
            int_msg.data = enemy_score;
            pub_enemy.publish(int_msg);
            
            bool_msg.data = arrived;
            pub_arrived.publish(bool_msg);
            
            uint8_msg.data = occupy_status;
            pub_occupy.publish(uint8_msg);
            
            std::cout << "已发送所有数据\n";
            continue;
        }
        
        int topic = std::stoi(input);
        int value;
        std::cin >> value;
        
        switch (topic)
        {
        case 1: 
            game_progress = value;
            std::cout << "设置游戏进度: " << value << " (0未开始,1进行中,2结束)\n";
            break;
            
        case 2: 
            hp = value;
            std::cout << "设置HP: " << value << " (0死亡,<100危险)\n";
            break;
            
        case 3: 
            bullet = value;
            std::cout << "设置子弹: " << value << " (<10不足)\n";
            break;
            
        case 4: 
            friendly_score = value;
            std::cout << "设置友方分数: " << value << "\n";
            break;
            
        case 5: 
            enemy_score = value;
            std::cout << "设置敌方分数: " << value << "\n";
            break;
            
        case 6: 
            arrived = (value != 0);
            std::cout << "设置到达状态: " << (arrived ? "已到达" : "未到达") << "\n";
            break;
        
        case 7: 
            if (value >= 0 && value <= 3)
            {
                occupy_status = value;
                std::string occupy_str[] = {"未占", "友方占", "敌方占", "双方占"};
                std::cout << "设置占领状态: " << occupy_str[value] << "\n";
            }
            else
            {
                std::cout << "占领状态范围: 0-3\n";
            }
            break;
            
        default:
            std::cout << "话题编号 1-7\n";
        }
    }
    
    return 0;
}