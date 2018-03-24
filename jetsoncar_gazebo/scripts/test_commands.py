#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64

speed = 0.3
steering_angle = 0.5

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    pub_vel_left_front_wheel = rospy.Publisher('/jetsoncar/front_left_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/jetsoncar/front_right_wheel_velocity_controller/command', Float64, queue_size=1)

    #pub_vel_left_rear_wheel = rospy.Publisher('/jetsoncar/rear_left_wheel_velocity_controller/command', Float64, queue_size=1)
    #pub_vel_right_rear_wheel = rospy.Publisher('/jetsoncar/rear_right_wheel_velocity_controller/command', Float64, queue_size=1)


    pub_pos_left_steering_hinge = rospy.Publisher('/jetsoncar/front_left_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/jetsoncar/front_right_hinge_position_controller/command', Float64, queue_size=1)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub_vel_left_front_wheel.publish(speed)
        pub_vel_right_front_wheel.publish(speed)
        #pub_vel_left_rear_wheel.publish(speed)
        #pub_vel_right_rear_wheel.publish(speed)
        pub_pos_left_steering_hinge.publish(steering_angle)
        pub_pos_right_steering_hinge.publish(steering_angle)
        rate.sleep()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
