#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import quaternion_from_euler


right_ticks = 0
left_ticks = 0

def right_ticks_callback(msg):
    global right_ticks
    right_ticks = msg.data

def left_ticks_callback(msg):
    global left_ticks
    left_ticks = msg.data

def compute_position():
    wheel_distance = 0.2075  # Distance between the wheels in meters
    wheel_radius = 0.035    # Wheel radius in meters
    ticks_per_revolution = 1795.92/2  # Number of ticks per wheel revolution

    # Compute the wheel circumference
    wheel_circumference = 2 * 3.141592 * wheel_radius

    # Compute the distance traveled by each wheel in meters
    right_distance = (right_ticks / float(ticks_per_revolution)) * wheel_circumference
    left_distance = (left_ticks / float(ticks_per_revolution)) * wheel_circumference

    # Compute the robot position
    position_x = (right_distance + left_distance) / 2
    position_y = 0
    orientation_theta = (right_distance - left_distance) / wheel_distance

    # Convert Euler angles to quaternion
    quaternion = quaternion_from_euler(0, 0, orientation_theta)

    # Create the Odometry message
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'

    # Set the position
    odom.pose.pose.position = Point(position_x, position_y, 0)
    odom.pose.pose.orientation = Quaternion(*quaternion)

    # Set the velocity
    odom.child_frame_id = 'base_link'
    odom.twist.twist.linear.x = 0
    odom.twist.twist.angular.z = 0

    return odom



if __name__ == '__main__':
    rospy.init_node('encoder_position_publisher', anonymous=True)

    rospy.Subscriber('/right_ticks', Int32, right_ticks_callback)
    rospy.Subscriber('/left_ticks', Int32, left_ticks_callback)

    pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        odom_msg = compute_position()
        pub.publish(odom_msg)
        rospy.loginfo("Published robot odometry: {}".format(odom_msg))
        rate.sleep()
