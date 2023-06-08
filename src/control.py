#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

MAX_X_SPEED = 0.8
MAX_Z_SPEED = 1


class ControlSystem:
    def __init__(self):
        rospy.init_node("control_system")
        self.velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber("/control_system/cmd_vel", Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        x = msg.linear.x
        z = msg.angular.z

        # limit speed to prevent accident
        if abs(x) > MAX_X_SPEED:
            if x > 0:
                msg.linear.x = MAX_X_SPEED
            elif x < 0:
                msg.linear.x = MAX_X_SPEED * -1

        if abs(z) > MAX_Z_SPEED:
            if z > 0:
                msg.angular.z = MAX_Z_SPEED
            elif z < 0:
                msg.angular.z = MAX_Z_SPEED * -1

        self.velocity_pub.publish(msg)


if __name__ == "__main__":
    try:
        contorlSystem = ControlSystem()
        rospy.spin()

    except KeyboardInterrupt:
        pass
