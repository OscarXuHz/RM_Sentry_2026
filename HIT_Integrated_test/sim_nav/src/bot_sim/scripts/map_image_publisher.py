#!/usr/bin/env python
"""Publish a PNG map image as nav_msgs/OccupancyGrid so it can be viewed in RViz."""
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid


def main():
    rospy.init_node("map_image_publisher", anonymous=False)

    image_path = rospy.get_param("~image_path", "")
    resolution = rospy.get_param("~resolution", 0.05)
    origin_x = rospy.get_param("~origin_x", 0.0)
    origin_y = rospy.get_param("~origin_y", 0.0)
    frame_id = rospy.get_param("~frame_id", "map")
    rate_hz = rospy.get_param("~rate", 1.0)
    # invert: if True, bright pixels = occupied (good for BEV height maps)
    # if False, dark pixels = occupied (good for occ maps where black = wall)
    invert = rospy.get_param("~invert", True)

    pub = rospy.Publisher("~grid", OccupancyGrid, queue_size=1, latch=True)

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        rospy.logerr("Failed to load image: %s" % image_path)
        return

    # OccupancyGrid row-0 = y_min (origin), but image row-0 = top = y_max.
    # Flip vertically so the grid aligns with world coordinates.
    img_flipped = np.flipud(img)

    if invert:
        # bright (255) -> 100 (occupied), dark (0) -> 0 (free)
        occ = (img_flipped.astype(np.float32) / 255.0 * 100.0).astype(np.int8)
    else:
        # dark (0) -> 100 (occupied), bright (255) -> 0 (free)
        occ = ((255 - img_flipped.astype(np.float32)) / 255.0 * 100.0).astype(np.int8)

    grid = OccupancyGrid()
    grid.header.frame_id = frame_id
    grid.info.resolution = resolution
    grid.info.width = img.shape[1]
    grid.info.height = img.shape[0]
    grid.info.origin.position.x = origin_x
    grid.info.origin.position.y = origin_y
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.w = 1.0
    grid.data = occ.flatten().tolist()

    rospy.loginfo(
        "Publishing %s (%dx%d, res=%.3f) at origin (%.3f, %.3f) on %s/grid"
        % (
            image_path,
            grid.info.width,
            grid.info.height,
            resolution,
            origin_x,
            origin_y,
            rospy.get_name(),
        )
    )

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        grid.header.stamp = rospy.Time.now()
        pub.publish(grid)
        rate.sleep()


if __name__ == "__main__":
    main()
