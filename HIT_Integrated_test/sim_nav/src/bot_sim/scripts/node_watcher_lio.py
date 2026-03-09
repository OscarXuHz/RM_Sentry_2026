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
        #decisions
        get_node_list()
        print("alive")

        #lidar
        if is_node_alive('livox_lidar_publisher2'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'livox_ros_driver2', 'msg_mixed.launch', '--no-summary'])
            rospy.loginfo("restarting msg_mixed.launch")
            time.sleep(1)
        
        #lidar
        if is_node_alive('threeD_lidar_merge'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'lidar_merge.launch', '--no-summary'])
            rospy.loginfo("restarting lidar_merge.launch")
            time.sleep(1)
        
        #lidar
        if is_node_alive('laserMapping'):
            pass
        else:
            subprocess.Popen(['roslaunch', '/home/sentry_train_test/AstarTraining/sim_nav/src/Point-LIO/launch/mapping_avia.launch'])
            rospy.loginfo("restarting point lio.launch")
            time.sleep(1)
        
        
        time.sleep(5)