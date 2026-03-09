#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
if(__name__ == '__main__'):
    rospy.init_node('test')
    Pub1 = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    Pub2 = rospy.Publisher('clicked_point', PointStamped, queue_size=10)
    rospy.sleep(1.0)
    rate = rospy.Rate(0.05)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()
    pose.pose.pose.position.x = 1.204344391822815
    pose.pose.pose.position.y = 2.883152723312378
    pose.pose.pose.position.z = 0.0
    pose.pose.pose.orientation.x = 0.0
    pose.pose.pose.orientation.y = 0.0
    pose.pose.pose.orientation.z = 0.2689035466926082
    pose.pose.pose.orientation.w = 0.9631671104103048
    pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    # Pub1.publish(pose)
    count = 1
    while not rospy.is_shutdown():
        count ^= 1
        pose2 = PointStamped()
        if(count & 1):
            pose2.header.frame_id = 'map'
            pose2.header.stamp = rospy.Time.now()
            pose2.point.x = 1.3168221712112427
            pose2.point.y = 2.8415262699127197
            pose2.point.z = 0.0
        else:
            pose2.header.frame_id = 'map'
            pose2.header.stamp = rospy.Time.now()
            pose2.point.x = 0.029756687581539154
            pose2.point.y = -1.8023000955581665
            pose2.point.z = 0.0
        Pub2.publish(pose2)
        rate.sleep()
# sentry_train_test@Astar:~/AstarTraining$ rostopic echo initialpose
# header: 
#   seq: 1
#   stamp: 
#     secs: 1716356828
#     nsecs: 517470254
#   frame_id: "map"
# pose: 
#   pose: 
#     position: 
#       x: 1.146601915359497
#       y: 2.4581284523010254
#       z: 0.0
#     orientation: 
#       x: 0.0
#       y: 0.0
#       z: 0.29807826417532696
#       w: 0.9545414335827566
#   covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
# ---
