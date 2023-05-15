#!/usr/bin/env python

import rospy
from marvelmind_nav.msg import hedge_pos_ang

# Parameters
max_speed = 0.5  # m/s
max_acceleration = 0.5  # m/s^2
duration = 5.0  # s
stop_duration = 2.0  # s
rotation_speed = 180.0 / stop_duration  # deg/s

def calculate_position(elapsed_time, duration, total_distance, stop_duration):
    half_duration = duration / 2.0
    t = elapsed_time % (2 * duration + 2 * stop_duration)
    
    if t <= stop_duration:
        position = 0
    elif t <= duration + stop_duration:
        position = total_distance * ((t - stop_duration) / duration)
    elif t <= duration + 2 * stop_duration:
        position = total_distance
    else:
        position = total_distance * (1 - ((t - (duration + 2 * stop_duration)) / duration))

    return position

def calculate_angle(elapsed_time, duration, stop_duration, rotation_speed):
    t = elapsed_time % (2 * duration + 2 * stop_duration)
    
    if t <= stop_duration:
        angle = 180 + rotation_speed * t
    elif t <= duration + stop_duration:
        angle = 0
    elif t <= duration + 2 * stop_duration:
        angle = rotation_speed * (t - duration - stop_duration)
    else:
        angle = 180

    return angle

def simulate_hedge_pos_ang():
    rospy.init_node('simulate_hedge_pos_ang', anonymous=True)

    hedge_pub = rospy.Publisher('/hedge_pos_ang', hedge_pos_ang, queue_size=10)

    rate = rospy.Rate(100)  # 100 Hz

    start_time = rospy.get_time()
    total_distance = max_speed * duration

    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        elapsed_time = current_time - start_time
        position = calculate_position(elapsed_time, duration, total_distance, stop_duration)
        angle = calculate_angle(elapsed_time, duration, stop_duration, rotation_speed)

        msg = hedge_pos_ang()

        msg.address = 1  # Mobile beacon address
        msg.timestamp_ms = int(current_time * 1000)  # Timestamp in milliseconds
        msg.x_m = position  # X coordinate in meters
        msg.y_m = 0  # Y coordinate in meters
        msg.z_m = 0  # Z coordinate in meters
        msg.flags = 0  # Flags of location

        # Orientation angle in degrees
        msg.angle = angle

        hedge_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        simulate_hedge_pos_ang()
    except rospy.ROSInterruptException:
        pass