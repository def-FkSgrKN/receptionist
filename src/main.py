#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
from geometry_msgs.msg import Twist

from receptionist.msg import Img_detect

from UDP_libs.UDP_server import UDP_Server

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
        
        self.img_argmax_w = 0 #最大矩形幅の人を追いかける
        
        self.img_detect_width_ave = 0 #幅の平均
        self.img_detect_x_mid_ave = 0 #中心の平均
        
        
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
     
     
    """
    人間追従 Follow me!
    """ 
    def follow_new_guest(self):
    
        
        while True:
                    
            twist = Twist()
            
            #何か写っている
            if len(self.img_detect_class_list) != 0:
                
                person_arg_w_list = []
                person_w_list = []
                
                #人のみの要素番号を追加する
                for i in range(len(self.img_detect_class_list)):
                    if self.img_detect_class_list[i] == self.PERSON_CLS_STR:
                        person_arg_w_list.append(i)
                        person_w_list.append(self.img_detect_width_list[i])
                    
                # 人が写っている
                if len(person_arg_w_list) != 0 and len(self.img_detect_class_list) != 0:
                
                    #最大値の要素番号を取得する。
                    person_w_max = max(person_w_list)
                    self.img_argmax_w = person_w_list.index(person_w_max)
                
                    
                    #総和
                    img_detect_width_sum = 0
                    img_detect_x_mid_sum = 0
                    
                    SAMPLING_NUM = 10 #平滑化標本数
                    for i in range(SAMPLING_NUM):
                        img_detect_width_sum += self.img_detect_width_list[self.img_argmax_w]
                        img_detect_x_mid_sum += self.img_detect_x_mid_list[self.img_argmax_w]

                    self.img_detect_width_ave = img_detect_width_sum / SAMPLING_NUM
                    self.img_detect_x_mid_ave = img_detect_x_mid_sum / SAMPLING_NUM
                
                    #速度を計算、変位の微分より x_now - x_before
                    #self.img_velocity_x_mid = self.img_detect_x_mid_list[self.img_argmax_w] - self.img_x_mid_before
                    
                    # 平滑化あり 
                    if self.img_detect_width_ave > self.WIDTH/2:
                        twist.linear.x = 0
                    
                    else:
                        twist.linear.x = -(self.WIDTH/2 - self.img_detect_width_ave) / self.WIDTH/3 * 2
                        
                    
                    if self.img_detect_x_mid_ave > self.WIDTH * 5/12 and self.img_detect_x_mid_ave < self.WIDTH * 7/12:
                        twist.angular.z = 0
                        
                    else:
                        twist.angular.z = -(self.img_detect_x_mid_ave - self.WIDTH/2) / (self.WIDTH/2) * 1.8

                    #コメントを外して、平滑化なし
                    """
                    if self.img_detect_width_list[self.img_argmax_w] > self.WIDTH/2:
                        twist.linear.x = 0
                    
                    else:
                        twist.linear.x = -(self.WIDTH/2 - self.img_detect_width_list[self.img_argmax_w]) / self.WIDTH/3 * 2
                        
                    
                    if self.img_detect_x_mid_list[self.img_argmax_w] > self.WIDTH * 5/12 and self.img_detect_x_mid_list[self.img_argmax_w] < self.WIDTH * 7/12:
                        twist.angular.z = 0
                        
                    else:
                        twist.angular.z = -(self.img_detect_x_mid_list[self.img_argmax_w] - self.WIDTH/2) / (self.WIDTH/2) * 1.8
                    """               
            
                    print("twist.linear.x=" + str(twist.linear.x) + ", twist.angular.z=" + str(twist.angular.z))
                    
                        
                    self.velocity_pub.publish(twist)
                    
                    
                    
    def detect_old_guests_and_chair(self):
        
        
            
        #コピーすることで、中のif文やwhile文のときに新しいデータに書き換えられ、listの長さが変わることを防ぐ
        
        img_detect_class_list = []
        img_detect_x_mid_list = []
        
        if len(self.img_detect_class_list) != 0:          
            img_detect_class_list = list(self.img_detect_class_list).copy()
            img_detect_x_mid_list = list(self.img_detect_x_mid_list).copy()
        
        #何か写っている
        if len(img_detect_class_list) != 0:
            
            #ロボットから見て左に映る順に並び替え
            #numpyのargsortを使う
            x_mid_ndarray = np.array(img_detect_x_mid_list)
            x_mid_ndarray_sorted_list = np.argsort(x_mid_ndarray).tolist()
            
            #人やゲストを並び替え  
            for i in range(len(img_detect_class_list)):
                print(str(img_detect_class_list[x_mid_ndarray_sorted_list[i]]) + ":" + str(img_detect_x_mid_list[x_mid_ndarray_sorted_list[i]]))
            """
            chair:369
            person:547
            """ 
            
            
            twist = Twist()
        
            
            #1. 一番左側に見えるものを追いかける
            while True: 
                tracking_x_mid = img_detect_x_mid_list[x_mid_ndarray_sorted_list[0]]
                
                if tracking_x_mid > self.WIDTH * 5/12 and tracking_x_mid < self.WIDTH * 7/12:
                    twist.angular.z = 0
                    break
                        
                else:
                    twist.angular.z = -(tracking_x_mid - self.WIDTH/2) / (self.WIDTH/2) * 1.8
            
            
            #2. 一番右側に見えるものを追いかける
            while True: 
                tail_idx = len(img_detect_class_list) - 1 #末尾の添字
                tracking_x_mid = img_detect_x_mid_list[x_mid_ndarray_sorted_list[tail_idx]]
                
                if tracking_x_mid > self.WIDTH * 5/12 and tracking_x_mid < self.WIDTH * 7/12:
                    twist.angular.z = 0
                    break
                        
                else:
                    twist.angular.z = -(tracking_x_mid - self.WIDTH/2) / (self.WIDTH/2) * 1.8
            
            
            #3. 中央の椅子を追いかける
            while True: 
                tracking_x_mid = img_detect_x_mid_list[x_mid_ndarray_sorted_list[1]]
                
                if tracking_x_mid > self.WIDTH * 5/12 and tracking_x_mid < self.WIDTH * 7/12:
                    twist.angular.z = 0
                    break
                        
                else:
                    twist.angular.z = -(tracking_x_mid - self.WIDTH/2) / (self.WIDTH/2) * 1.8
                
            print("\n")
            
            #関数内関数、添字番号の物体を追いかける
            def track_thing(index):
                while True: 
                    tracking_x_mid = img_detect_x_mid_list[x_mid_ndarray_sorted_list[index]]
                    
                    if tracking_x_mid > self.WIDTH * 5/12 and tracking_x_mid < self.WIDTH * 7/12:
                        twist.angular.z = 0
                        break
                            
                    else:
                        twist.angular.z = -(tracking_x_mid - self.WIDTH/2) / (self.WIDTH/2) * 1.8  
                
                    
    
        
     
    def main_ROS(self):       
        """
        椅子と古参のゲストの集合写真を撮影する
        """
        
        #self.follow_new_guest()   
        
        self.detect_old_guests_and_chair()
       
                
                
                    

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