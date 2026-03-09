#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, Point

class ClickedPointsPublisher:
    def __init__(self, points_stamped):
        self.points_stamped = points_stamped
        self.pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
        delta_t = 20
        self.rate = rospy.Rate(1/delta_t)  # 1 Hz

    def publish_points(self):
        while not rospy.is_shutdown():
            for point_stamped in self.points_stamped:
                self.pub.publish(point_stamped)
                self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('clicked_points_publisher')
    points_xy = [[-1.3,8.4],[0.0,0.0]]
    points_stamped = []
    for i in range(len(points_xy)):
        point = PointStamped()
        point.header.frame_id = "map"
        point.header.stamp = rospy.Time.now()
        point.point = Point()
        point.point.x = points_xy[i][0]
        point.point.y = points_xy[i][1]
        point.point.z = 0
        points_stamped.append(point)
    cpp = ClickedPointsPublisher(points_stamped)
    cpp.publish_points()