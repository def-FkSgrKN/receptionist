#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Twist

from UDP_libs.UDP_server import UDP_Server

class Main:
    def __init__(self):
        rospy.init_node("main")
               
        # control
        self.velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
        
        # udp
        self.IP_ADRESS = "127.0.0.1"
        self.PORT = 8890
        self.UDP_SERVER = UDP_Server(self.IP_ADRESS, self.PORT)
        
        self.img_addr = ""
        
    
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
        
        
        """
        椅子と古参のゲストの集合写真を撮影する
        """
        
        print("L49")
        
        while True:
            #画像クライアントから住所をもらう
            #print("L53")
            recv_from_img_msgs, addr = self.UDP_SERVER.get_message()
            
            #"print("L56")
            
            #print("addr=" + str(addr))
            print("recv_from_img_msgs" + str(recv_from_img_msgs))
            
            if recv_from_img_msgs == "img_init":
                self.img_addr = addr
                break
            
        
        #time.sleep(2)
        
        start_time = time.time() #開始時刻
        TURN_TIME = 3
        
        
        #画像クライアントへ写真を取るという指示を出す。
        send_to_img_msgs = "S1_memorize"
        self.UDP_SERVER.send_message(send_to_img_msgs, self.img_addr)
        
        self.control(TURN_TIME/2, angular_z=0.7)
        time.sleep(1)
        self.control(TURN_TIME, angular_z=-0.7)       
        
        
if __name__ == "__main__":
    main_instance = Main()
    main_instance.main()