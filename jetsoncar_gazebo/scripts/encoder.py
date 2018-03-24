#!/usr/bin/env python

'''
Script to parse /jetsoncar/joint_states topic and publish encoder values to /encoder/front and /encoder/rear
'''


import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import numpy as np
import math
import tf2_ros

class EncoderNode:
    # Set publishers
    pub_front = rospy.Publisher('/encoder/front', Int32, queue_size=1)
    pub_rear = rospy.Publisher('/encoder/rear', Int32, queue_size=1)
    pub_timestamp = rospy.Publisher('/encoder/timestamp', Int32, queue_size=1)

    def __init__(self):
        # Subscribe to joint_states
        rospy.Subscriber('/jetsoncar/joint_states', JointState, self.joint_states_update)

    def joint_states_update(self, msg):
        recieved_timestamp = rospy.Time.now()
	# Find the index of the wheels
        try:
            idxFrontLeft = msg.name.index('front_left_wheel_joint')
            idxFrontRight = msg.name.index('front_right_wheel_joint')
            idxRearLeft = msg.name.index('rear_left_wheel_joint')
            idxRearRight = msg.name.index('rear_right_wheel_joint')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract encoder angles in radians
            encFrontLeft = msg.position[idxFrontLeft]
            encFrontRight = msg.position[idxFrontLeft]
            encRearLeft = msg.position[idxRearLeft]
            encRearRight = msg.position[idxRearRight]

            # Convert radians into ticks using a gearing ratio of 40 and 12 ticks pr. rev
            encFrontLeft = (encFrontLeft / (2*math.pi)) * (40*12)
            encFrontRight = (encFrontRight / (2*math.pi)) * (40*12)
            encRearLeft = (encRearLeft / (2*math.pi)) * (40*12)
            encRearRight = (encRearRight / (2*math.pi)) * (40*12)

            # Prepare the data for publishing
            encFront = Int32()
            encFront.data = (encFrontLeft + encFrontRight) / 2
            encRear = Int32()
            encRear.data = (encRearLeft + encRearRight) / 2
            timestamp = Int32()
            timestamp.data = (recieved_timestamp.secs * 1000) + (recieved_timestamp.nsecs / 1000000)  # publish milliseconds

            self.pub_front.publish(encFront)
            self.pub_rear.publish(encRear)
            self.pub_timestamp.publish(timestamp)

# Start the node
if __name__ == '__main__':
    rospy.init_node("jetsoncar_encoder_node")
    node = EncoderNode()
    rospy.spin()
