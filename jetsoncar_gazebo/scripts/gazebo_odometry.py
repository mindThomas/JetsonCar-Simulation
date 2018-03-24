#!/usr/bin/env python

'''
Script to publish car base_link transform relative to the map origo using the Gazebo pose of the base_link
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros

class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

    def __init__(self):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None
        self.right_wheel_pose = Pose()
        self.left_wheel_pose = Pose()

        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the jetsoncar
        try:
            arrayIndex = msg.name.index('jetsoncar::base_link')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]

        # Find the index of the jetsoncar
        try:
            arrayIndex = msg.name.index('jetsoncar::rear_right_wheel')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.right_wheel_pose = msg.pose[arrayIndex]

        try:
            arrayIndex = msg.name.index('jetsoncar::rear_left_wheel')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.left_wheel_pose = msg.pose[arrayIndex]

        self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = 'map'
        cmd.child_frame_id = 'base_link'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        self.pub_odom.publish(cmd)

        tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

        tf = TransformStamped(
            header=Header(
                frame_id='map',
                stamp=self.last_recieved_stamp
            ),
            child_frame_id='rear_right_wheel',
            transform=Transform(
                translation=self.right_wheel_pose.position,
                rotation=self.right_wheel_pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

        tf = TransformStamped(
            header=Header(
                frame_id='map',
                stamp=self.last_recieved_stamp
            ),
            child_frame_id='rear_left_wheel',
            transform=Transform(
                translation=self.left_wheel_pose.position,
                rotation=self.left_wheel_pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()
