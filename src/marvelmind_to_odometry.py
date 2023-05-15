#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_pos_ang, hedge_imu_fusion


def hedge_pos_ang_callback(msg):
    odom = Odometry()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'

    # Set the position
    odom.pose.pose.position = Point(msg.x_m, msg.y_m, 0.0)

    # Convert orientation angle to quaternion
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = 0.0
    quaternion.w = 1.0

    odom.pose.pose.orientation = quaternion

    # Set the velocity
    odom.child_frame_id = 'base_link'
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    # Set the covariance values
    # The covariance values should be provided as a list of 36 elements representing a 6x6 matrix.
    # Here, we assume a diagonal matrix with equal variance for simplicity.
    covariance = [0.01,  0.0,  0.0,  0.0,  0.0,  0.0,
                  0.0,  0.01,  0.0,  0.0,  0.0,  0.0,
                  0.0,   0.0, 0.01,  0.0,  0.0,  0.0,
                  0.0,   0.0,  0.0,  0.1,  0.0,  0.0,
                  0.0,   0.0,  0.0,  0.0,  0.1,  0.0,
                  0.0,   0.0,  0.0,  0.0,  0.0,  0.1]  # Example: using 0.1 as the covariance value for all elements

    # Populate the covariance matrix in the odometry message
    odom.pose.covariance = covariance  # Covariance for position and orientation

    # Publish the odometry message
    odom_pub.publish(odom)


def hedge_imu_fusion_callback(msg):
    imu_msg = Imu()

    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'base_link'

    # Assuming that qw, qx, qy, qz represent the quaternion for orientation
    imu_msg.orientation = Quaternion(msg.qx, msg.qy, msg.qz, msg.qw)

    imu_msg.angular_velocity = Vector3(msg.vx, msg.vy, msg.vz)

    # Assuming that ax, ay, az represent the linear acceleration
    imu_msg.linear_acceleration = Vector3(msg.ax, msg.ay, msg.az)

    # Publish the IMU message
    imu_pub.publish(imu_msg)


if __name__ == '__main__':
    rospy.init_node('marvelmind_to_odometry', anonymous=True)

    rospy.Subscriber('/hedge_pos_ang', hedge_pos_ang, hedge_pos_ang_callback)
    rospy.Subscriber('/hedge_imu_fusion', hedge_imu_fusion, hedge_imu_fusion_callback)

    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)

    rospy.spin()
