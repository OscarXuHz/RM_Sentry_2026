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
def check_all_nodes():
    get_node_list()
    print("alive")
    if is_node_alive('map_server'):
        pass
    else:
        subprocess.Popen(['roslaunch', 'bot_sim', 'map_server.launch', '--no-summary'])
        rospy.loginfo("restarting map_server.launch")
        time.sleep(1)
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
    if is_node_alive('threeD_lidar_merge_pointcloud'):
        pass
    else:
        subprocess.Popen(['roslaunch', 'bot_sim', 'lidar_merge_pointcloud.launch', '--no-summary'])
        rospy.loginfo("restarting lidar_merge_pointcloud.launch")
        time.sleep(1)
    if is_node_alive('hdl_localization_nodelet'):
        pass
    else:
        subprocess.Popen(['roslaunch', 'hdl_localization', 'hdl_localization_uc.launch', '--no-summary'])
        rospy.loginfo("restarting hdl_localization.launch")
        time.sleep(1)
    #real_robot
    if is_node_alive('real_robot_transform'):
        pass
    else:
        subprocess.Popen(['roslaunch', 'bot_sim', 'real_robot_transform.launch', '--no-summary'])
        rospy.loginfo("restarting real_robot_transform.launch")
        time.sleep(1)
    #lidar_filter
    if is_node_alive('threeD_lidar_filter'):
        pass
    else:
        subprocess.Popen(['roslaunch', 'bot_sim', 'lidar_filter.launch', '--no-summary'])
        rospy.loginfo("restarting lidar_filter.launch")
        time.sleep(1)
    #dbscan_bfs_3D
    if is_node_alive('dbscan_bfs_3D'):
        pass
    else:
        subprocess.Popen(['roslaunch', 'bot_sim', 'dbscan_bfs_3D.launch', '--no-summary'])
        rospy.loginfo("restarting dbscan_bfs_3D.launch")
        time.sleep(1)
    #ser2msg
    if is_node_alive('ser2msg_decision_givepoint'):
        pass
    else:
        subprocess.Popen(['roslaunch', 'bot_sim', 'ser2msg_tf_decision_givepoint.launch', '--no-summary'])
        rospy.loginfo("restarting ser2msg_tf_decision.launch")
        time.sleep(1)
    # dstarlite
    if is_node_alive('dstarlite'):
        pass
    else:
        subprocess.Popen(['roslaunch', 'bot_sim', 'dstarlite.launch'])
        rospy.loginfo("restarting dstarlite.launch")
        time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('node_watcher')
    # time.sleep(30)
    currentTime=datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    subprocess.Popen(['mkdir', '-p', '/home/sentry_train_test/recording/'+currentTime])
    subprocess.Popen(['rosbag', 'record', '-a', '--split', '--duration=10s', '--chunksize=10', '-O', '/home/sentry_train_test/recording/'+currentTime+'/'+currentTime])
    rospy.loginfo("starting rosbag")
    time.sleep(1)
    
    # check_all_nodes()
    # subprocess.Popen(['rviz', '-d', '/home/sentry_train_test/AstarTraining/sim_nav/src/bot_sim/config/rviz/tf_test.rviz'])
    while rospy.is_shutdown() == False:
        check_all_nodes()

        time.sleep(2)