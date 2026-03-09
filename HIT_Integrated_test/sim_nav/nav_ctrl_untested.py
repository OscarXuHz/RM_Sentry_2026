#!/usr/bin/env python3

from typing import List
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Twist
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf2_ros
import actionlib
from bot_sim.msg import NavActionAction, NavActionGoal, NavActionFeedback, NavActionResult
import numpy as np

from bot_sim.srv import RRTStar

import sys
import os

script_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"DWA"))
if script_path not in sys.path:
    sys.path.append(script_path)
rospy.loginfo("Added path: %s", script_path)
from traj import trajObject, TrajectoryMode
from dwa_util import MapUtil
from test_util_2 import PathPublisher, PointStampedPublisher, PointListPublisher
from cost_function import CostFunction


class NavCtrl:
    _feedback = NavActionFeedback()
    _result = NavActionResult()
    def __init__(self, trajectories: List[trajObject], cost_map_path: str, dyn_map_name: str,
                 dt: float, traj_cut_percentage: float = 0.5, iter_percentage: float = 0.5,
                 pathfollowext_percentage: float = 0.8, cmd_vel_topic: str = "/cmd_vel",
                 within_point: float = 0.6, within_goal: float = 0.4, cost_method: str = "omni"):
        self._action_name = "nav_ctrl"
        self._as = actionlib.SimpleActionServer(self._action_name, NavActionAction, execute_cb=self.nav_to_goal, auto_start = False)
        self._as.start()

        self.trajectories = trajectories
        # Cut trajectory points to only keep remote points
        self.traj_cut_index = int(traj_cut_percentage * len(trajectories[0].traj_poses))
        for traj in trajectories:
            traj.traj_poses = traj.traj_poses[self.traj_cut_index:]
        # These value used for cost function
        # path_iter is used for bearing cost and turn cost
        # pathfollowext_iter is used for path following cost
        # obstacbleext_iter is used for obstacle avoidance cost
        path_iter = int(iter_percentage * len(trajectories[0].traj_poses))
        pathfollowext_iter = int(pathfollowext_percentage * len(trajectories[0].traj_poses))
        obsext_iter = len(trajectories[0].traj_poses)-1

        self.cost_func = CostFunction(cost_map_path, dyn_map_name,path_iter, pathfollowext_iter, obsext_iter)
        if cost_method == "omni":
            self.cost = self.cost_func.total_cost_omni
        else:
            self.cost = self.cost_func.total_cost

        # transform as robot location
        self.loc_trans = None
        self.robot_frame = rospy.get_param("~robot_frame")
        self.world_frame = rospy.get_param("~world_frame")
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(120.0)) # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Path Control
        self.path = []
        self.path_len = -1
        
        # Update Next Point Condition
        self.within_point = within_point
        self.next_point_index = 0

        # Goal Reach Condition
        self.within_goal = within_goal

        # other
        self.cmd_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.dt = dt
        self.vel_scaler = 2
        self.loc_tran = None
    
    def call_rrt_star(self, curr_x: float, curr_y: float, goal_x: float, goal_y: float):
        rospy.wait_for_service('rrt_star')
        try:
            rrt_star = rospy.ServiceProxy('rrt_star', RRTStar)
            resp = rrt_star(curr_x, curr_y, goal_x, goal_y)
            if resp.success == 1:
                self.path_len = len(resp.path)
                self.path = resp.path
                return 1
            else:
                return 0
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return 0
    
    def update_location(self):
        try:
            self.loc_tran = self.tf_buffer.lookup_transform(self.world_frame, self.robot_frame,
                                                    rospy.Time.now(), rospy.Duration(10.0))#写成两个param
            for traj in self.trajectories:
                traj.CalcWorldFramePoses(self.loc_tran.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No location found")
            rospy.logerr(self.world_frame)
            rospy.logerr(self.robot_frame)
    
    def get_next_point(self, curr_x: float, curr_y: float, goal_x: float, goal_y: float):
        if self.next_point_index < self.path_len:
            next_point = self.path[self.next_point_index]
            if abs(next_point.x - curr_x) < self.within_point \
                and abs(next_point.y - curr_y) < self.within_point:
                self.next_point_index += 1
                return self.get_next_point(curr_x, curr_y, goal_x, goal_y)
            else:
                return next_point
        else:
            return Point(goal_x, goal_y, 0)
    
    def check_best_traj_vel(self, next_point: Point):
        min_cost = float('inf')
        best_traj = None
        scaler = np.sqrt((next_point.x - self.loc_tran.transform.translation.x)**2 + \
                         (next_point.y - self.loc_tran.transform.translation.y)**2)
        for traj in self.trajectories:
            cost = self.cost(traj.world_frame_traj_poses, traj.traj_poses, next_point, scaler)
            if cost < min_cost:
                min_cost = cost
                best_traj = traj
        return best_traj.x_vel, best_traj.y_vel, best_traj.omega, best_traj
    
    def nav_to_goal(self, goal):
        self.next_point_index = 0
        while self.loc_tran == None:
            self.update_location()
        if self.loc_tran == None:
            rospy.logerr("No location found")
            return -1
        else:
            zero_progress = np.sqrt((goal.goal_x - self.loc_tran.transform.translation.x)**2 +\
                                     (goal.goal_y - self.loc_tran.transform.translation.y)**2)
            dist_to_goal = zero_progress
            
        # path_get = self.call_rrt_star(self.loc_tran.transform.translation.x, 
        #                                self.loc_tran.transform.translation.y, goal.goal_x, goal.goal_y)
        path_get = 1
        self.path = [Point(goal.goal_x, goal.goal_y, 0)]
        self.path_len = 1
        if path_get != 1:
            rospy.logerr("path not found")
            return -1
        else:
            # path publisher
            path_publisher = PathPublisher()
            path_publisher.calc_path_from_point_list(self.path)
            # publish point
            next_point_publisher = PointStampedPublisher('next_point_topic')
            point_list_publisher = PointListPublisher(frame_id="map")
            best_traj_publisher = PointListPublisher(frame_id="map",topic_name="best_traj_points",
                                                     r=0.0,b=0.0,g=1.0,a=1.0,
                                                     x_scale=0.05,y_scale=0.05,z_scale=0.05)

            rate = rospy.Rate(1/self.dt)
            vel = Twist()
            scaler = self.vel_scaler
            while dist_to_goal > self.within_goal and not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    vel.linear.x = 0
                    vel.linear.y = 0
                    vel.angular.z = 0
                    self.cmd_publisher.publish(vel)
                    return 0
                next_point = self.get_next_point(self.loc_tran.transform.translation.x, 
                                                 self.loc_tran.transform.translation.y, 
                                                 goal.goal_x, goal.goal_y)
                pub_point = next_point
                pub_point.z = 0
                next_point_publisher.publish_point(pub_point)

                x_vel, y_vel, omega, best_traj = self.check_best_traj_vel(next_point)
                best_traj_point_list = []
                for point in best_traj.world_frame_traj_poses:
                    best_traj_point_list.append(Point(point[0][0], point[1][0], 0.15))
                best_traj_publisher.publish_point_list(best_traj_point_list)

                vel.linear.x = x_vel/scaler
                vel.linear.y = -y_vel/scaler
                # if next_point == Point(goal.goal_x, goal.goal_y, 0):
                #     vel.linear.x = x_vel/scaler / 2
                #     vel.linear.y = -y_vel/scaler/ 2

                vel.angular.z = omega
                self.cmd_publisher.publish(vel)

                traj_point_list = []
                for traj in self.trajectories:
                    for point in traj.world_frame_traj_poses:
                        traj_point_list.append(Point(point[0][0], point[1][0], 0.1))
                point_list_publisher.publish_point_list(traj_point_list)

                path_publisher.publish_path()

                self.update_location()
                dist_to_goal = np.sqrt((goal.goal_x - self.loc_tran.transform.translation.x)**2 +\
                                        (goal.goal_y - self.loc_tran.transform.translation.y)**2)
                self._feedback.progress_bar = dist_to_goal / zero_progress
                self._as.publish_feedback(self._feedback)
                rate.sleep()

            rospy.loginfo("goal reached")
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0
            self.cmd_publisher.publish(vel)
            self.cmd_publisher.publish(vel)
            self.cmd_publisher.publish(vel)
            self._result.result = 1
            self._as.set_succeeded(self._result)
            return 1

    def nav_to_goal_rrt(self, goal):
        self.next_point_index = 0
        while self.loc_tran == None:
            self.update_location()
        if self.loc_tran == None:
            rospy.logerr("No location found")
            return -1
        else:
            zero_progress = np.sqrt((goal.goal_x - self.loc_tran.transform.translation.x)**2 +\
                                     (goal.goal_y - self.loc_tran.transform.translation.y)**2)
            dist_to_goal = zero_progress
            
        path_get = self.call_rrt_star(self.loc_tran.transform.translation.x, 
                                       self.loc_tran.transform.translation.y, goal.goal_x, goal.goal_y)
        if path_get != 1:
            path_get = 1
            self.path = [Point(goal.goal_x, goal.goal_y, 0)]
            self.path_len = 1
        if path_get == 1:
            # path publisher
            path_publisher = PathPublisher()
            path_publisher.calc_path_from_point_list(self.path)
            # publish point
            next_point_publisher = PointStampedPublisher('next_point_topic')
            point_list_publisher = PointListPublisher(frame_id="map")
            best_traj_publisher = PointListPublisher(frame_id="map",topic_name="best_traj_points",
                                                     r=0.0,b=0.0,g=1.0,a=1.0,
                                                     x_scale=0.05,y_scale=0.05,z_scale=0.05)

            rate = rospy.Rate(1/self.dt)
            vel = Twist()
            scaler = self.vel_scaler
            while dist_to_goal > self.within_goal and not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    vel.linear.x = 0
                    vel.linear.y = 0
                    vel.angular.z = 0
                    self.cmd_publisher.publish(vel)
                    return 0
                next_point = self.get_next_point(self.loc_tran.transform.translation.x, 
                                                 self.loc_tran.transform.translation.y, 
                                                 goal.goal_x, goal.goal_y)
                pub_point = next_point
                pub_point.z = 0
                next_point_publisher.publish_point(pub_point)

                x_vel, y_vel, omega, best_traj = self.check_best_traj_vel(next_point)
                best_traj_point_list = []
                for point in best_traj.world_frame_traj_poses:
                    best_traj_point_list.append(Point(point[0][0], point[1][0], 0.15))
                best_traj_publisher.publish_point_list(best_traj_point_list)

                vel.linear.x = x_vel/scaler
                vel.linear.y = -y_vel/scaler
                # if next_point == Point(goal.goal_x, goal.goal_y, 0):
                #     vel.linear.x = x_vel/scaler / 2
                #     vel.linear.y = -y_vel/scaler/ 2

                vel.angular.z = omega
                self.cmd_publisher.publish(vel)

                traj_point_list = []
                for traj in self.trajectories:
                    for point in traj.world_frame_traj_poses:
                        traj_point_list.append(Point(point[0][0], point[1][0], 0.1))
                point_list_publisher.publish_point_list(traj_point_list)

                path_publisher.publish_path()

                self.update_location()
                dist_to_goal = np.sqrt((goal.goal_x - self.loc_tran.transform.translation.x)**2 +\
                                        (goal.goal_y - self.loc_tran.transform.translation.y)**2)
                self._feedback.progress_bar = dist_to_goal / zero_progress
                self._as.publish_feedback(self._feedback)
                rate.sleep()

            rospy.loginfo("goal reached")
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0
            self.cmd_publisher.publish(vel)
            self.cmd_publisher.publish(vel)
            self.cmd_publisher.publish(vel)
            self._result.result = 1
            self._as.set_succeeded(self._result)
            return 1
        
if __name__=="__main__":
    rospy.init_node('nav_ctrl')

    dt = 0.2
    # trajectories = TrajectoryMode.DifferentialDriveTrajectory(x_vel=0.5, omega_increment=0.1, num_of_traj_one_side=7) + \
    #     TrajectoryMode.DifferentialDriveTrajectory(x_vel=-0.5, omega_increment=0.1, num_of_traj_one_side=7)
    trajectories = TrajectoryMode.ShiftTrajectories(linear_vel=1.3, num_of_traj=25) + \
            TrajectoryMode.ShiftTrajectories(linear_vel=1.0, num_of_traj=20) + \
            TrajectoryMode.ShiftTrajectories(linear_vel=0.6, num_of_traj=15) + \
            TrajectoryMode.ShiftTrajectories(linear_vel=1.6, num_of_traj=21)

    for traj in trajectories:
        TrajectoryMode.GenTrajectory(traj, 0.05, record_every_iter=1, iteration=10)
    
    cost_map_path = rospy.get_param("~cost_map_path")
    dyn_map_name = rospy.get_param("~dyn_map_name")    
    NavCtrl = NavCtrl(trajectories, cost_map_path, dyn_map_name, dt, traj_cut_percentage=0.1)

    rospy.spin()
    

