#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped

def drive_straight():
    # Initialize the node
    rospy.init_node('drive_straight_node', anonymous=True)
    
    # Create publisher - note: NO vehicle name in topic
    pub = rospy.Publisher('car_cmd', Twist2DStamped, queue_size=1)
    
    # Wait for publisher to connect
    rospy.sleep(1)
    
    # Create the message
    cmd_msg = Twist2DStamped()
    cmd_msg.v = 0.3  # Linear velocity (m/s) - adjust this
    cmd_msg.omega = 0.0  # Angular velocity (rad/s) - 0 for straight
    
    # Publish for a duration to drive ~1m
    rate = rospy.Rate(10)  # 10 Hz
    duration = rospy.Duration(3.5)  # Adjust based on velocity
    start_time = rospy.Time.now()
    
    while rospy.Time.now() - start_time < duration:
        pub.publish(cmd_msg)
        rate.sleep()
    
    # Stop the robot
    cmd_msg.v = 0.0
    pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        drive_straight()
    except rospy.ROSInterruptException:
        pass
