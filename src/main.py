#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Twist

from receptionist.msg import Img_detect

from UDP_libs.UDP_server import UDP_Server

from image36 import get_image_size

class Main:
    def __init__(self):
        rospy.init_node("main")
        
        # image
        rospy.Subscriber("/image_detect", Img_detect, self.image_detect_callback)
        self.Img_detect_rosmsg = Img_detect()
        
        self.img_detect_count = 0
        self.img_detect_class_list = []
        self.img_detect_x_mid_list = []
        self.img_detect_y_mid_list = []
        self.img_detect_width_list = []
        self.img_detect_height_list = []
        
        #幅:640、高さ:480
        self.WIDTH = 640
        self.HEIGHT = 480
        
        self.PERSON_CLS_STR = "person"
        self.CHAIR_CLS_STR = "chair"
               
               
        # control
        self.velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
        self.linear_x = 0
        self.angler_z = 0
        
        
        # udp
        self.IP_ADRESS = "127.0.0.1"
        self.PORT = 8890
        self.UDP_SERVER = UDP_Server(self.IP_ADRESS, self.PORT)
        
        self.img_addr = ""
        
        
    #callback関数
    def image_detect_callback(self, msg):
        self.img_detect_count = msg.count
        self.img_detect_class_list = msg.class_list
        self.img_detect_x_mid_list = msg.x_mid_list
        self.img_detect_y_mid_list = msg.y_mid_list
        self.img_detect_width_list = msg.width_list
        self.img_detect_height_list = msg.height_list
        
        #self.person_detect_distance = msg.data
    
    
    def control(self, twist, move_time, linear_x=0, angular_z=0):
        start_time = time.time() #開始時刻
        
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        while True:
            end_time = time.time() #終了時刻
            #print(end_time - start_time)
            
            self.velocity_pub.publish(twist)
            
            if end_time - start_time > move_time:
                break
     
     
    def main_ROS(self):       
        """
        椅子と古参のゲストの集合写真を撮影する
        """
        
        move_time = 10
        start_time = time.time() #開始時刻
        while True:
            end_time = time.time() #終了時刻
            
            #if end_time - start_time > move_time:
            #    break
            
            twist = Twist()
            
            #print("self.img_detect_count=" + str(self.img_detect_count))
            
            
            if len(self.img_detect_class_list) == 1:
                
                #print("self.img_detect_count=" + str(self.img_detect_count))
                #print("self.img_detect_class_list[0]" + str(self.img_detect_class_list[0]))
            
                if self.img_detect_class_list[0] == self.PERSON_CLS_STR:
                    
                    if self.img_detect_width_list[0] > self.WIDTH/2:
                        twist.linear.x = 0
                    
                    else:
                        twist.linear.x = -(self.WIDTH/2 - self.img_detect_width_list[0]) / self.WIDTH/3 * 2
                        
                    
                    if self.img_detect_x_mid_list[0] > self.WIDTH * 5/12 and self.img_detect_x_mid_list[0] < self.WIDTH * 7/12:
                        twist.angular.z = 0
                        
                    else:
                        twist.angular.z = -(self.img_detect_x_mid_list[0] - self.WIDTH/2) / (self.WIDTH/2) * 1.8
                        
            
                    print("twist.linear.x=" + str(twist.linear.x) + ", twist.angular.z=" + str(twist.angular.z))
                        
                    self.velocity_pub.publish(twist)
                    
                    

        exit()
            
        
        #time.sleep(2)
        
        start_time = time.time() #開始時刻
        TURN_TIME = 3
        
        
        #画像クライアントへ写真を取るという指示を出す。
        #send_to_img_msgs = "S1_memorize"
        #self.UDP_SERVER.send_message(send_to_img_msgs, self.img_addr)
        
        twist = Twist()
        
        self.control(twist, TURN_TIME/2, angular_z=0.7)
        time.sleep(1)
        self.control(twist, TURN_TIME, angular_z=-0.7) 
        
        
    
    def main_UDP(self):
               
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
    #main_instance.main_UDP()
    main_instance.main_ROS()