#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class LocalFrameBridge:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_local_topic = rospy.get_param("~odom_local_topic", "/odom_local")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.clicked_topic = rospy.get_param("~clicked_topic", "/clicked_point")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.center_on_first_odom = rospy.get_param("~center_on_first_odom", True)
        self.passthrough = rospy.get_param("~passthrough", False)
        self.map_size_x = rospy.get_param("~map_size_x", 0.0)
        self.map_size_y = rospy.get_param("~map_size_y", 0.0)
        self.map_lower_x = rospy.get_param("~map_lower_x", 0.0)
        self.map_lower_y = rospy.get_param("~map_lower_y", 0.0)

        self.aligned_in = rospy.get_param("~aligned_in", "/aligned_points")
        self.aligned_out = rospy.get_param("~aligned_out", "/aligned_points_local")
        self.filtered_in = rospy.get_param("~filtered_in", "/filted_topic_3d")
        self.filtered_out = rospy.get_param("~filtered_out", "/filted_topic_3d_local")

        self.offset_set = False
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0

        self.odom_pub = rospy.Publisher(self.odom_local_topic, Odometry, queue_size=10)
        self.clicked_pub = rospy.Publisher(self.clicked_topic, PointStamped, queue_size=10)
        self.aligned_pub = rospy.Publisher(self.aligned_out, PointCloud2, queue_size=1)
        self.filtered_pub = rospy.Publisher(self.filtered_out, PointCloud2, queue_size=1)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb, queue_size=10)
        rospy.Subscriber(self.aligned_in, PointCloud2, self.aligned_cb, queue_size=1)
        rospy.Subscriber(self.filtered_in, PointCloud2, self.filtered_cb, queue_size=1)

        rospy.loginfo(
            "local_frame_bridge: odom %s -> %s, goal %s -> %s",
            self.odom_topic,
            self.odom_local_topic,
            self.goal_topic,
            self.clicked_topic,
        )

    def _compute_offset(self, odom_x, odom_y, odom_z):
        """Compute offset so that the robot maps to the map center."""
        if self.passthrough:
            self.offset_x = 0.0
            self.offset_y = 0.0
            self.offset_z = 0.0
        elif self.center_on_first_odom and self.map_size_x > 0.0 and self.map_size_y > 0.0:
            center_x = self.map_lower_x + self.map_size_x * 0.5
            center_y = self.map_lower_y + self.map_size_y * 0.5
            self.offset_x = odom_x - center_x
            self.offset_y = odom_y - center_y
            self.offset_z = odom_z
        else:
            self.offset_x = odom_x
            self.offset_y = odom_y
            self.offset_z = odom_z
        rospy.loginfo(
            "local_frame_bridge: set offset (%.3f, %.3f, %.3f) -> local (%.3f, %.3f)",
            self.offset_x, self.offset_y, self.offset_z,
            odom_x - self.offset_x, odom_y - self.offset_y,
        )

    def _ensure_offset(self, msg: Odometry):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        odom_z = msg.pose.pose.position.z

        if not self.offset_set:
            self._compute_offset(odom_x, odom_y, odom_z)
            self.offset_set = True
            return

        # Re-center if the robot would fall outside the map (skip in passthrough)
        if not self.passthrough and self.map_size_x > 0.0 and self.map_size_y > 0.0:
            local_x = odom_x - self.offset_x
            local_y = odom_y - self.offset_y
            margin = 0.5  # re-center before hitting the very edge
            if (local_x < self.map_lower_x + margin
                    or local_x > self.map_lower_x + self.map_size_x - margin
                    or local_y < self.map_lower_y + margin
                    or local_y > self.map_lower_y + self.map_size_y - margin):
                rospy.logwarn(
                    "local_frame_bridge: robot local pos (%.2f, %.2f) near map edge, re-centering",
                    local_x, local_y,
                )
                self._compute_offset(odom_x, odom_y, odom_z)

    def odom_cb(self, msg: Odometry):
        self._ensure_offset(msg)
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.pose.pose.position.x -= self.offset_x
        out.pose.pose.position.y -= self.offset_y
        out.pose.pose.position.z -= self.offset_z
        out.twist = msg.twist
        self.odom_pub.publish(out)

    def goal_cb(self, msg: PoseStamped):
        if not self.offset_set:
            # Wait for odom to establish offset
            return
        pt = PointStamped()
        pt.header = msg.header
        if not pt.header.frame_id:
            pt.header.frame_id = self.frame_id
        pt.point.x = msg.pose.position.x - self.offset_x
        pt.point.y = msg.pose.position.y - self.offset_y
        pt.point.z = msg.pose.position.z - self.offset_z
        self.clicked_pub.publish(pt)

    def _rebase_cloud(self, msg: PointCloud2) -> PointCloud2:
        if not self.offset_set:
            return None
        fields = list(msg.fields)
        field_names = [f.name for f in fields]
        try:
            ix = field_names.index("x")
            iy = field_names.index("y")
            iz = field_names.index("z")
        except ValueError:
            return None

        rebased_points = []
        for pt in pc2.read_points(msg, field_names=field_names, skip_nans=False):
            pt_list = list(pt)
            pt_list[ix] = pt_list[ix] - self.offset_x
            pt_list[iy] = pt_list[iy] - self.offset_y
            pt_list[iz] = pt_list[iz] - self.offset_z
            rebased_points.append(tuple(pt_list))

        out = pc2.create_cloud(msg.header, fields, rebased_points)
        out.header.frame_id = self.frame_id
        return out

    def aligned_cb(self, msg: PointCloud2):
        out = self._rebase_cloud(msg)
        if out is not None:
            self.aligned_pub.publish(out)

    def filtered_cb(self, msg: PointCloud2):
        out = self._rebase_cloud(msg)
        if out is not None:
            self.filtered_pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("local_frame_bridge")
    LocalFrameBridge()
    rospy.spin()
