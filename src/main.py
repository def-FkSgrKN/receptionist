#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Twist

class Main:
    def __init__(self):
        rospy.init_node("main")
               
        # control
        self.velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
        
    
    def control(self, move_time, linear_x=0, angular_z=0):
        start_time = time.time() #開始時刻
        twist = Twist()
        
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        while True:
            end_time = time.time() #終了時刻
            #print(end_time - start_time)
            
            self.velocity_pub.publish(twist)
            
            if end_time - start_time > move_time:
                break
            
    
    def main(self):
        
        time.sleep(2)
        
        start_time = time.time() #開始時刻
        TURN_TIME = 2
        
        """
        椅子と古参のゲストの集合写真を撮影する
        """
        self.control(2, angular_z=1)
        time.sleep(1)
        self.control(2, angular_z=-1)       
        
        
if __name__ == "__main__":
    main_instance = Main()
    main_instance.main()