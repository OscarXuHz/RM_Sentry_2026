#!/usr/bin/env python3
import subprocess
import time
import rospy
from datetime import datetime

def get_node_list():
    global node_list
    node_list=subprocess.check_output(['rosnode', 'list'])
def is_node_alive(node_name):
    global node_list
    if(node_name in str(node_list)):
        return True
    # print(output)
    return False
    # finally:
    #     return False

if __name__ == '__main__':
    rospy.init_node('node_watcher')
    # time.sleep(30)
    currentTime=datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    subprocess.Popen(['mkdir', '/home/sentry_train_test/recording/'+currentTime])
    subprocess.Popen(['rosbag', 'record', '-a', '--split', '--duration=10s', '--chunksize=10', '-O', '/home/sentry_train_test/recording/'+currentTime+'/'+currentTime])
    rospy.loginfo("starting rosbag")
    time.sleep(1)


    while rospy.is_shutdown() == False:
        get_node_list()
        print("alive")


        if is_node_alive('imu_filter'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'imu_filter.launch', '--no-summary'])
            rospy.loginfo("restarting imu_filter.launch")
            time.sleep(1)

        #lidar
        if is_node_alive('livox_lidar_publisher2'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'livox_ros_driver2', 'msg_mixed.launch', '--no-summary'])
            rospy.loginfo("restarting msg_mixed.launch")
            time.sleep(1)

        if is_node_alive('threeD_lidar_merge_pointcloud_test'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'lidar_merge_pointcloud_test.launch', '--no-summary'])
            rospy.loginfo("restarting lidar_merge_pointcloud_test.launch")
            time.sleep(1)

        if is_node_alive('hdl_localization_nodelet'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'hdl_localization', 'hdl_localization_jiantu.launch', '--no-summary'])
            rospy.loginfo("restarting hdl_localization.launch")
            time.sleep(1)

        
        time.sleep(5)