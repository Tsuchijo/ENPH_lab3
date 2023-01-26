#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 

## Move Robot node
# use ros to publish to the topic /cmd_vel:
def move_robot():
    rospy.init_node('topic_publisher', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.1
        pub.publish(twist)
        rate.sleep()

move_robot()