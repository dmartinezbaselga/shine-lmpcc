#!/usr/bin/env python3
import os
import struct
import random
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
import argparse
import time
import copy

MIN_Y = -2.5
MAX_Y = 3.7
MAX_X = 3.5
MIN_X = -4.0
dir_path = os.path.dirname(os.path.realpath(__file__))

class Goal:
    def __init__(self, x=0., y=0.) -> None:
        self.x = x
        self.y = y

    def distance(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

def generate_random_goals(n_goals):
    with open(dir_path + '/random_goals.bin', 'wb') as f:

        # prev_was_x = int(0)
        # prev_was_negative = int(1)
        prev_goal = Goal(0., 0.)
        for _ in range(n_goals):
            goal = copy.deepcopy(prev_goal)
            while prev_goal.distance(goal) < (MAX_X - MIN_X) / 3. * 2.:
                # print(prev_goal.distance(goal))
                # print((MAX_X - MIN_X) / 3. * 2.)
                is_x = random.randint(0, 1)
                is_negative = random.randint(0, 1)
                if is_x:
                    x = random.random() * (MAX_X - MIN_X) + MIN_X
                    if is_negative:
                        y = MIN_Y + 0.5
                    else:
                        y = MAX_Y - 0.5
                else:
                    y = random.random() * (MAX_Y - MIN_Y) + MIN_Y
                    if is_negative:
                        x = MIN_X + 0.5
                    else:
                        x = MAX_X - 0.5
                goal = Goal(x, y)
            # print("Accept")

            # x = random.random()*(MAX_X-MIN_X) + MIN_X
            # y = random.random()*(MAX_Y-MIN_Y) + MIN_Y
            data = struct.pack('d', x)
            f.write(data)
            data = struct.pack('d', y)
            f.write(data)
            prev_goal = copy.deepcopy(goal)
            # print(x, y)

class GoalsPublisher:
    def __init__(self, random_goals, max_goals) -> None:
        rospy.init_node('goal_publisher')
        self.max_goals = max_goals
        self.num_goals_reached = 0
        self.random_file_name = 'random_goals.bin'
        self.random_goals = random_goals

        self.robot_pose_sub_ = rospy.Subscriber('/Robot_1/pose', PoseStamped, self.robot_pose_callback)
        self.goal_pub_ = rospy.Publisher('/roadmap/goal', PoseStamped, queue_size=10)
        self.ref_path_pub_ = rospy.Publisher('/roadmap/reference', Path, queue_size=10)

        self.last_pub_stamp_ = rospy.Time.now()
        self.min_goal = True

        self.min_dist_goal = 0.8
        if random_goals:
            self.goals_file = open(dir_path + '/random_goals.bin', 'rb')
        self.current_goal = None
        rospy.spin()
    
    def read_goal(self, robot_x, robot_y):
        goal = Goal()
        if self.random_goals:
            data = self.goals_file.read(8)
            goal.x = struct.unpack('d', data)[0]
            data = self.goals_file.read(8)
            goal.y = struct.unpack('d', data)[0]
        elif math.sqrt((MIN_X-robot_x)**2 +(MIN_Y-robot_y)**2) > 1.0:
            goal.x = MIN_X
            goal.y = MIN_Y
        else:
            goal.x = MAX_X
            goal.y = MAX_Y
        self.min_goal = not self.min_goal
        # print(goal.x, goal.y)
        return goal

    def publish_current_goal(self):
        msg = PoseStamped()
        msg.pose.position.x = self.current_goal.x
        msg.pose.position.y = self.current_goal.y
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        self.goal_pub_.publish(msg)

    def construct_reference_path(self, msg):

        robot_x = msg.pose.position.x
        robot_y = msg.pose.position.y

        path = Path()

        # Add the start
        new_g = PoseStamped()
        new_g.pose.position.x = robot_x
        new_g.pose.position.y = robot_y
        new_g.header.stamp = msg.header.stamp
        new_g.header.frame_id = 'map'
        path.poses.append(new_g)
        theta = math.atan2(self.current_goal.y - robot_y, self.current_goal.x - robot_x)

        # Add the rest
        while math.sqrt((path.poses[-1].pose.position.x - self.current_goal.x)**2 + (path.poses[-1].pose.position.y - self.current_goal.y)**2) > 2.0:
            new_g = PoseStamped()
            new_g.pose.position.x = path.poses[-1].pose.position.x + 0.5 * math.cos(theta)
            new_g.pose.position.y = path.poses[-1].pose.position.y + 0.5 * math.sin(theta)
            new_g.header.stamp = msg.header.stamp
            new_g.header.frame_id = 'map'
            path.poses.append(new_g)

        # Add the goal
        new_g = PoseStamped()
        new_g.pose.position.x = self.current_goal.x
        new_g.pose.position.y = self.current_goal.y
        new_g.header.stamp = msg.header.stamp
        new_g.header.frame_id = 'map'
        path.poses.append(new_g)

        # Extend
        for i in range(4):
            new_g = PoseStamped()
            new_g.pose.position.x = path.poses[-1].pose.position.x + 0.5 * math.cos(theta)
            new_g.pose.position.y = path.poses[-1].pose.position.y + 0.5 * math.sin(theta)
            new_g.header.stamp = msg.header.stamp
            new_g.header.frame_id = 'map'
            path.poses.append(new_g)

        path.header.frame_id = 'map'
        path.header.stamp = msg.header.stamp

        self.reference_path = path

    def publish_path(self):
        # print(f"GoalPublisher: Publishing reference path for goal ({self.current_goal.x}, {self.current_goal.y})")
        # print("-----------")
        # for p in self.reference_path.poses:
        #     print(f"Point (x = {p.pose.position.x}, y = {p.pose.position.y})")
        self.ref_path_pub_.publish(self.reference_path)
    
    def robot_pose_callback(self, msg: PoseStamped):
        # TODO: Filter poses
        if self.current_goal:
            # @Note: Robot radius subtracted
            goal_dist = -0.325 + math.sqrt((msg.pose.position.x - self.current_goal.x)**2 + (msg.pose.position.y - self.current_goal.y)**2)
            # print(f"Goal distance: {goal_dist}")
        if self.current_goal is None or goal_dist < self.min_dist_goal:
            self.num_goals_reached += 1
            if (self.num_goals_reached >= self.max_goals - 1 and self.random_goals):
                print("Configured number of goals reached! Exiting.")
                rospy.signal_shutdown("Goals reached")
            else:
                self.current_goal = self.read_goal(msg.pose.position.x, msg.pose.position.y)
                self.publish_current_goal()
                self.construct_reference_path(msg)
                self.publish_path()
                self.last_pub_stamp_ = msg.header.stamp

        if self.last_pub_stamp_ + rospy.Duration(1. / 20.) <= msg.header.stamp:
            self.publish_path()
            self.publish_current_goal()
            self.last_pub_stamp_ = msg.header.stamp


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Parse configuration file')
    parser.add_argument('--n_goals', type=int, required=False, default=10)
    parser.add_argument('--publish', default=False, action='store_true')
    args, unknown = parser.parse_known_args()
    print(args)
    if args.publish:
        goal_publisher = GoalsPublisher(False, args.n_goals)
    else:
        generate_random_goals(args.n_goals)