#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

class Main:
    def __init__(self):
        rospy.init_node("main")
               
        # control
        self.control_vel_pub = rospy.Publisher("/control_system/cmd_vel", Twist, queue_size=1)
        
    
    def main(self):
        t = Twist()
        
        t.angular.z = -0.4
        t.linear.x = 1
        
        self.control_vel_pub.publish(t)
        
if __name__ == "__main__":
    main_instance = Main()
    main_instance.main()