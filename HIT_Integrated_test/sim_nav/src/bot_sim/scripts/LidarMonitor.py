#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import threading
import subprocess
import time
from livox_ros_driver2.msg import CustomMsg

class LidarMonitor:
    def __init__(self, topic_name, node_executable, timeout=10.0):
        self.topic_name = topic_name
        self.node_executable = node_executable
        self.timeout = timeout
        print(timeout)
        self.last_received = rospy.get_time()
        self.timer = threading.Timer(self.timeout, self.check_timeout)
        
        # Subscribe to the topic
        rospy.Subscriber(self.topic_name, CustomMsg, self.callback)
        self.timer.start()

    def callback(self, msg):
        self.last_received = rospy.get_time()
        # Reset the timer
        self.timer.cancel()
        self.timer = threading.Timer(self.timeout, self.check_timeout)
        self.timer.start()

    def check_timeout(self):
        # Time since the last received message
        time_since_last_received = rospy.get_time() - self.last_received
        print(time_since_last_received)
        rospy.loginfo(f"time_since_last_received: {time_since_last_received}")
        if time_since_last_received >= self.timeout:
            rospy.loginfo(f"No messages on {self.topic_name} for {self.timeout} seconds. Restarting node.")
            time.sleep(1)
            self.restart_node()
        else:
            rospy.loginfo(f"Received message on {self.topic_name} within timeout.")

    def restart_node(self):
        # Terminate the existing node if necessary
        # Restart the node using the node executable
        subprocess.call(['shutdown','now'])

if __name__ == '__main__':
    rospy.init_node('lidar_monitor')
    
    # Parameters for the topic to monitor and the node to restart
    topic_to_monitor1 = rospy.get_param("~topic_to_monitor1", "/livox/lidar_192_168_1_141")
    node_executable1 = rospy.get_param("~node_executable1", "livox_lidar_publisher2")
    timeout_duration1 = rospy.get_param("~timeout_duration1", 10.0)  # in seconds

    LidarMonitor(topic_to_monitor1, node_executable1, timeout_duration1)

    topic_to_monitor2 = rospy.get_param("~topic_to_monitor2", "/livox/lidar_192_168_1_124")
    node_executable2 = rospy.get_param("~node_executable2", "livox_lidar_publisher2")
    timeout_duration2 = rospy.get_param("~timeout_duration2", 10.0)  # in seconds

    LidarMonitor(topic_to_monitor2, node_executable2, timeout_duration2)

    rospy.spin()
