#!/usr/bin/env python3

import rospy
from livox_ros_driver2.msg import CustomMsg
import math
import struct
def angle_between_vectors_3d(v1, v2):
    dot_product = sum(a*b for a, b in zip(v1, v2))
    norm_a = math.sqrt(sum(a*a for a in v1))
    norm_b = math.sqrt(sum(b*b for b in v2))
    if(norm_a ==0 or norm_b ==0):
        return 0
    return math.acos(dot_product / (norm_a * norm_b))

new_cloud = CustomMsg()
converted = False
angle_limit=math.pi / 3.4
max_accept_angle=math.pi*2 - angle_limit
min_accept_angle=angle_limit
def get_lidar(msg):
    global new_cloud
    global converted
    new_cloud = CustomMsg()
    new_cloud.header = msg.header
    new_cloud.timebase = msg.timebase
    new_cloud.point_num = 0
    new_cloud.lidar_id = msg.lidar_id
    new_cloud.rsvd = msg.rsvd
    # new_cloud.data = msg.data
    new_cloud.points = []
    temp = msg.points[0]
    for i in msg.points:
        # Unpack the data for each field
        x, y, z = i.x, i.y, i.z
        # Append the data to the corresponding lists
        if(angle_between_vectors_3d([0,-1,0],[x,y,z])<max_accept_angle and angle_between_vectors_3d([0,-1,0],[x,y,z])>min_accept_angle):
            new_cloud.points.append(i) 
            new_cloud.point_num += 1
    converted = True


if __name__ == "__main__":
    rospy.init_node("lidar_converter")
    sub = rospy.Subscriber("/livox/lidar", CustomMsg, get_lidar, queue_size=100)
    DeltaTime = 0.01
    rate = rospy.Rate(1/DeltaTime)
    pub = rospy.Publisher("/new_lidar", CustomMsg, queue_size=100)
    while not rospy.is_shutdown():
        if converted is True:
            pub.publish(new_cloud)
            converted = False
        rate.sleep()
