#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist


def twist_publisher():
    rospy.init_node('twist_publisher', anonymous=True)
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        r = 5
        pi = 3.14159
        t = 10
        expect_z = 2*pi/t
        expect_x = 2*pi*r/t
        twist_msg = Twist()
        twist_msg.linear.x = expect_x
        twist_msg.angular.z = expect_z
        twist_pub.publish(twist_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        twist_publisher()
    except rospy.ROSInterruptException:
        pass
