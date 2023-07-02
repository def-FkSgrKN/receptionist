#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
import statistics

from geometry_msgs.msg import Twist

from receptionist.msg import Img_detect, Img_take_pictures

from UDP_libs.UDP_server import UDP_Server

#control 
from control_libs.point_to_point import PointToPoint


#sound
from speech_and_NLP.src.textToSpeech import textToSpeech #発話
from speech_and_NLP.src.speechToText import recognize_speech #音声認識
from speech_and_NLP.src.speechToText import recognize_speech #音声認識
from speech_and_NLP.src.tools.speech_to_text.extractPersonNameEnglish import extract_names#与えた文から名前を文字列で返す
from speech_and_NLP.src.tools.speech_to_text.extractFavoriteDrinks import extract_favorite_drinks#与えた文から好きな飲み物の名前を文字列で返す

import speech_recognition as sr
import pyaudio

from audio_libs.BERT_QandA_exp import roberta_model_set, roberta_model_res

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
        
        
        self.image_pub = rospy.Publisher("/image_take_pictures", Img_take_pictures, queue_size=1)
        self.Img_take_pictures_rosmsg = Img_take_pictures()
        
        
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
        self.IP_ADRESS = "127.0.0.10"
        self.PORT = 8890
        #self.UDP_SERVER = UDP_Server(self.IP_ADRESS, self.PORT)
        
        self.img_addr = ""
        
        
    #callback関数
    def image_detect_callback(self, msg):
        self.img_detect_count = msg.count
        self.img_detect_class_list = msg.class_list
        self.img_detect_x_mid_list = msg.x_mid_list
        self.img_detect_y_mid_list = msg.y_mid_list
        self.img_detect_width_list = msg.width_list
        self.img_detect_height_list = msg.height_list
        
        
    
    #時間制御の関数
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
     
     
    """
    人間追従 Follow me!
    """ 
    def follow_new_guest(self):
    
        stop_start_time = 0
        STOP_DELTA_TIME = 2

        while True:
                    
            twist = Twist()
            
            stop_end_time = time.time()

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
                    if self.img_detect_width_ave > self.WIDTH * 1/2:
                        twist.linear.x = 0
                    
                    else:
                        twist.linear.x = -(self.WIDTH * 1/2 - self.img_detect_width_ave) / self.WIDTH/3 * 2
                        
                    
                    if self.img_detect_x_mid_ave > self.WIDTH * 5/12 and self.img_detect_x_mid_ave < self.WIDTH * 7/12:
                        twist.angular.z = 0
                        
                    else:
                        twist.angular.z = -(self.img_detect_x_mid_ave - self.WIDTH/2) / (self.WIDTH/2) * 1.8

                    #終了処理
                    if twist.angular.z == 0 and twist.linear.x == 0:
                        break


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
                    


                #人が写っていない場合
                else:
                    twist.angular.z = 0.8
                    twist.linear.x = 0
                
            #ものでさえ写っていない場合
            else:
                twist.angular.z = 0.8
                twist.linear.x = 0
            
                        
            self.velocity_pub.publish(twist)
                    
            
    """
    ゲストを招待する
    (椅子追従)
    """
    def invite_new_guest(self):
        
        nearest_chair_identify_count = 0
        
        
        while True:
                    
            twist = Twist()
            
            
            
            #書き換え防止
            img_detect_class_list = []
            img_detect_width_list = []
            img_detect_x_mid_list = []
            
            if len(self.img_detect_class_list) != 0:   
                img_detect_class_list = list(self.img_detect_class_list).copy()
                img_detect_width_list = list(self.img_detect_width_list).copy()
                img_detect_x_mid_list = list(self.img_detect_x_mid_list).copy()
                
                
            #何か写っている
            if len(img_detect_class_list) != 0:
                
                chair_arg_w_list = []
                chair_w_list = []
                
                #椅子のみの要素番号を追加する
                for i in range(len(img_detect_class_list)):
                    if img_detect_class_list[i] == self.CHAIR_CLS_STR:
                        chair_arg_w_list.append(i)
                        chair_w_list.append(img_detect_width_list[i])
                    
                # 椅子が写っている
                if len(chair_arg_w_list) != 0:
                    
                    #追いかける椅子の添字を一回だけ調べる
                    if nearest_chair_identify_count == 0:
                
                        #最大値の要素番号を取得する。
                        chair_w_max = max(chair_w_list)
                        self.img_argmax_w = chair_w_list.index(chair_w_max)
                        
                        nearest_chair_identify_count += 1
                
                
                    #配列がもとの添字の値以下になった場合、もう一度調べる
                    if len(chair_arg_w_list) <= self.img_argmax_w:
                        
                        #最大値の要素番号を取得する。
                        chair_w_max = max(chair_w_list)
                        self.img_argmax_w = chair_w_list.index(chair_w_max)
                        
                    
                    
                    #総和
                    img_detect_width_sum = 0
                    img_detect_x_mid_sum = 0
                    
                    SAMPLING_NUM = 10 #平滑化標本数
                    for i in range(SAMPLING_NUM):
                        img_detect_width_sum += img_detect_width_list[self.img_argmax_w]
                        img_detect_x_mid_sum += img_detect_x_mid_list[self.img_argmax_w]

                    self.img_detect_width_ave = img_detect_width_sum / SAMPLING_NUM
                    self.img_detect_x_mid_ave = img_detect_x_mid_sum / SAMPLING_NUM
                
                    #速度を計算、変位の微分より x_now - x_before
                    #self.img_velocity_x_mid = self.img_detect_x_mid_list[self.img_argmax_w] - self.img_x_mid_before
                    
                    # 平滑化あり 
                    if self.img_detect_width_ave > self.WIDTH * 2/3:
                        twist.linear.x = 0
                    
                    else:
                        twist.linear.x = -(self.WIDTH/2 - self.img_detect_width_ave) / self.WIDTH/3 * 2
                        
                    
                    if self.img_detect_x_mid_ave > self.WIDTH * 5/12 and self.img_detect_x_mid_ave < self.WIDTH * 7/12:
                        twist.angular.z = 0
                        
                    else:
                        twist.angular.z = -(self.img_detect_x_mid_ave - self.WIDTH/2) / (self.WIDTH/2) * 1.5

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
        
        nearest_chair_identify_count = 0 #一定回数以上あれば終了するための変数
        nearest_chair_identify_idx_list = [] #最も近い椅子を最頻値で決める
        tracking_chair_idx = 0
        
        find_count = 0

        #椅子の矩形が3分の1になるまで微調整
        while True:
            twist = Twist()
            
            img_detect_class_list = []
            img_detect_x_mid_list = []
            img_detect_width_list = []
            
            if len(self.img_detect_class_list) != 0:          
                img_detect_class_list = list(self.img_detect_class_list).copy()
                img_detect_x_mid_list = list(self.img_detect_x_mid_list).copy()
                img_detect_width_list = list(self.img_detect_width_list).copy()
            
            
            
            chair_idx_list = []
            chair_x_mid_list = []
            chair_width_list = []
                
            if len(img_detect_class_list) != 0: 
                for i in range(len(img_detect_class_list)):
                    if img_detect_class_list[i] == self.CHAIR_CLS_STR:
                        chair_idx_list.append(i)
                        chair_x_mid_list.append(img_detect_x_mid_list[i])
                        chair_width_list.append(img_detect_width_list[i])

                        find_count = 1 #椅子が見つかったら回転しないようにする
                    
                #print("chair_idx_list=" + str(chair_idx_list))
                #print("chair_x_mid_list=" + str(chair_x_mid_list))
                #print("chair_width_list=" + str(chair_width_list))

                
                if len(chair_idx_list) > tracking_chair_idx:
                
                    #初回20回は追いかける椅子を調べる            
                    if nearest_chair_identify_count < 20:
                    
        
                        max_chair_width = max(chair_width_list)
                        max_chair_index = chair_width_list.index(max_chair_width)
                        nearest_chair_identify_idx_list.append(max_chair_index)
                        
                        #仮の追いかける椅子
                        if nearest_chair_identify_count == 1:
                            tracking_chair_idx = max_chair_index
                        
                        
                        nearest_chair_identify_count += 1
                        
                        time.sleep(0.1)
                        
                        
                    elif nearest_chair_identify_count == 10:
                        tracking_chair_idx = statistics.mode(nearest_chair_identify_idx_list)
                        nearest_chair_identify_count += 1
                        
                        
                    
                    if len(chair_width_list) >= tracking_chair_idx:
                        
                        max_chair_width = max(chair_width_list)
                        tracking_chair_idx = chair_width_list.index(max_chair_width)
                    
                    
                    #十分に近づいていた場合    
                    if chair_width_list[tracking_chair_idx] > self.WIDTH/3 - 30 and chair_width_list[tracking_chair_idx] < self.WIDTH/3 + 50:
                        break
                    
                    else:
                        twist.linear.x = -(self.WIDTH/2 - chair_width_list[tracking_chair_idx]) / self.WIDTH/3 * 2
                        twist.angular.z = -(chair_x_mid_list[tracking_chair_idx] - self.WIDTH/2) / (self.WIDTH/2) * 1.5
            
                else:
                    time.sleep(1)

                    """
                    if find_count == 0:
                        twist.angular.z = 0.8
                        twist.linear.x = 0

                    elif find_count == 1:
                        twist.angular.z = 0
                        twist.linear.x = 0.1
                    """

            else:
                time.sleep(1)

                if find_count == 0:
                    twist.angular.z = 0.8
                    twist.linear.x = 0

                elif find_count == 1:
                        twist.angular.z = 0
                        twist.linear.x = 0.1
                
            
                        
            self.velocity_pub.publish(twist)     
                    
                        
                        
                        
                        
                        

            
        tracking_state = 1
        
        turn_start_time = 0
        turn_end_time = 0
        turn_delta_time = 0
        
        detect_two_things_count = 0
    

        #最頻値を調べるリスト
        left_thing_class_list = []
        mid_thing_class_list = []
        right_thing_class_list = []


        LMR_thing_class_list = ["chair", "chair", "chair"]


        #視界に人、椅子を含め3つになるまで下がる
        while True:
            if len(self.img_detect_class_list) != 3:
                break


        #exit()


        """
        realsenseを使用する場合、視野が広いためすべてを視野に含めやすい

        """
        while True:

            twist = Twist()
                
            #コピーすることで、中のif文やwhile文のときに新しいデータに書き換えられ、listの長さが変わることを防ぐ
            
            img_detect_class_list = []
            img_detect_x_mid_list = []
            
            #状態4で終了
            if tracking_state == 4:
                break
            
            #書き換え防止
            if len(self.img_detect_class_list) != 0:          
                img_detect_class_list = list(self.img_detect_class_list).copy()
                img_detect_x_mid_list = list(self.img_detect_x_mid_list).copy()

            #すべて視界に写ったとき
            if len(img_detect_class_list) == 3:

                #ロボットから見て左に映る順に並び替え
                #numpyのargsortを使う
                x_mid_ndarray = np.array(img_detect_x_mid_list)
                x_mid_ndarray_sorted_list = np.argsort(x_mid_ndarray).tolist()


                print("左:" + img_detect_class_list[x_mid_ndarray_sorted_list[0]])
                print("中:" + img_detect_class_list[x_mid_ndarray_sorted_list[1]])
                print("右:" + img_detect_class_list[x_mid_ndarray_sorted_list[2]])

                chair_name = "left"

                for i in len(img_detect_class_list):
                    
                    #番号が若い順に椅子がある
                    if img_detect_class_list[x_mid_ndarray_sorted_list[i]] == "chair":

                        if i == 0:
                            chair_name = "left"

                        elif i == 1:
                            chair_name = "mid"

                        elif i == 2:
                            chair_name = "right"

                        break

                #ゲストを着席させる
                inviting_new_guest_txt = "Please sit down the " + chair_name + " chair."
                textToSpeech(inviting_new_guest_txt, gTTS_lang='en')
                        



        exit()

        """
        内蔵カメラや小型usbカメラを使用する場合、視野が狭いためすべてを視野に含めるのは難しい
        """
        while True:
                
            twist = Twist()
                
            #コピーすることで、中のif文やwhile文のときに新しいデータに書き換えられ、listの長さが変わることを防ぐ
            
            img_detect_class_list = []
            img_detect_x_mid_list = []
            
            #状態4で終了
            if tracking_state == 4:
                break
            
            
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
                #for i in range(len(img_detect_class_list)):
                #    print(str(img_detect_class_list[x_mid_ndarray_sorted_list[i]]) + ":" + str(img_detect_x_mid_list[x_mid_ndarray_sorted_list[i]]))
                """
                chair:369
                person:547
                """ 
        
                #75[cm]離れたときにちょうどよく動く
            
                #print("tracking_state=" + str(tracking_state))
                #1. 一番左側に見えるものを追いかける
                if tracking_state == 1: 
                    
                    detect_two_things_count = 0
                    
                    tail_idx = len(img_detect_class_list) - 1 #末尾の添字
                    tracking_x_mid = img_detect_x_mid_list[x_mid_ndarray_sorted_list[tail_idx]]
                    
                    #一番左に見える椅子が画面の左端に来るまで回転する
                    if tracking_x_mid <= 1/6 * self.WIDTH:
                        print("OK")
                        tracking_state = 2
                        turn_start_time = time.time()
                    
                        """
                        #目標角度が近くなったときに通りすぎないようにする
                        elif tracking_x_mid > 1/6 * self.WIDTH and tracking_x_mid < 2/6:
                            twist.angular.z = -0.5
                            print("tracking_x_mid=" + str(tracking_x_mid))
                    
                        """
                        """
                        elif tracking_x_mid <= 2/6 * self.WIDTH:
                            print("左端:" + str(img_detect_class_list[x_mid_ndarray_sorted_list[tail_idx]]))
                            left_thing_class_list.append(img_detect_class_list[x_mid_ndarray_sorted_list[tail_idx]])

                            twist.angular.z = -1
                        """                    
                            
                    elif tracking_x_mid > 1/2 * self.WIDTH and len(img_detect_class_list) >= 2:
                        print("左端:" + str(img_detect_class_list[x_mid_ndarray_sorted_list[tail_idx]]))
                        left_thing_class_list.append(img_detect_class_list[x_mid_ndarray_sorted_list[tail_idx]])

                        twist.angular.z = -1
                        
                        
                    else:
                        twist.angular.z = -1
                        #print("tracking_x_mid=" + str(tracking_x_mid))
                                    
                
               

                #2. 一番右側に見えるものを追いかける
                elif tracking_state == 2: 
                    #print("img_detect_x_mid_list=" + str(img_detect_x_mid_list))
                    tracking_x_mid = img_detect_x_mid_list[x_mid_ndarray_sorted_list[0]]
                    
                    
                    #一番左に見える椅子が画面の左端に来るまで回転する
                    if tracking_x_mid >= 5/6 * self.WIDTH:
                        print("OK")
                        turn_end_time = time.time() #左回転の終了時刻
                        turn_delta_time = turn_end_time - turn_start_time #左回転にかかった時間を計測
                        
                        turn_start_time = time.time() #スタート時刻を初期化 右回転の開始時刻
                        tracking_state = 3
                    
                        """    
                        #目標角度が近くなったときに通りすぎないようにする
                        elif tracking_x_mid > 5/6 * self.WIDTH and tracking_x_mid < 5/6:
                            twist.angular.z = 0.5
                            print("tracking_x_mid=" + str(tracking_x_mid)) 
                        """
                        
                    #elif tracking_x_mid > 3/5 * self.WIDTH and tracking_x_mid <= 4/5 * self.WIDTH:
                    #    print("右端:" + str(img_detect_class_list[x_mid_ndarray_sorted_list[0]]))
                    #    twist.angular.z = 0.6
                    
                    elif tracking_x_mid >= 4/5 * self.WIDTH:
                        twist.angular.z = 1
                    
                    
                    #2つの物体が写っているうちの前半部分に着目すると
                    elif len(img_detect_class_list) >= 2 and tracking_x_mid < 1/2 * self.WIDTH:
                        print("中央:" + str(img_detect_class_list[x_mid_ndarray_sorted_list[1]]))
                        print("右端:" + str(img_detect_class_list[x_mid_ndarray_sorted_list[0]]))
                        mid_thing_class_list.append(img_detect_class_list[x_mid_ndarray_sorted_list[1]])
                        right_thing_class_list.append(img_detect_class_list[x_mid_ndarray_sorted_list[0]])

                        detect_two_things_count += 1
                        twist.angular.z = 1
                        
                    else:
                        twist.angular.z = 1
                        #print("tracking_x_mid=" + str(tracking_x_mid))
                        
                        
                #3. 中央の椅子を追跡する。
                elif tracking_state == 3: 
                    
                    tracking_x_mid = img_detect_x_mid_list[x_mid_ndarray_sorted_list[0]]
                    
                    turn_end_time = time.time() #右回転の終了時刻
                
                    #一番左に見える椅子が画面の左端に来るまで回転する
                    """
                    if tracking_x_mid < 1/9 * self.WIDTH and len(img_detect_x_mid_list) >= 2:
                        print("OK")
                        tracking_state = 4
                    """
                    #print("turn_delta_time=" + str(turn_delta_time))
                    #半分の時刻でできると考える
                    if turn_end_time - turn_start_time > turn_delta_time/2:
                        print("OK")
                        tracking_state = 4                 
                               
                    else:
                        twist.angular.z = -1
                        #print("tracking_x_mid=" + str(tracking_x_mid))

        
                #print("\n")
                #print("twist.angular.z=" + str(twist.angular.z))    
            
            else:
                if tracking_state == 1:
                    twist.angular.z = 0.7
                    twist.linear.x = 0.15
                
                elif tracking_state == 2:
                    twist.angular.z = -0.7
                    twist.linear.x = 0.15
                    
                time.sleep(0.1)
            
                
                    
            self.velocity_pub.publish(twist)                
        
        #左端、中央、右端の物体のクラスの変数
        if len(left_thing_class_list) != 0: 
            LMR_thing_class_list[0] = statistics.mode(left_thing_class_list)

        if len(mid_thing_class_list) != 0:
            LMR_thing_class_list[1] = statistics.mode(mid_thing_class_list)

        if len(right_thing_class_list) != 0:
            LMR_thing_class_list[2] = statistics.mode(right_thing_class_list)

        print("左:" + LMR_thing_class_list[0] + "、中:" + LMR_thing_class_list[1] + "、右:" + LMR_thing_class_list[2])

        """
        どこに新しいゲストを案内するかを決める
        """

        
     
    def main_ROS(self):       
        """
        椅子と古参のゲストの集合写真を撮影する
        """
        
        time.sleep(10)
        
        #for i in range(3):
        textToSpeech("follow_new_guest", gTTS_lang='en')
        #self.follow_new_guest()   
            
        #MOVETIME = 7
        #ANG_Z = 0.8
        #self.control(move_time=MOVETIME, angular_z=ANG_Z)
        
        time.sleep(2)
        
        textToSpeech("control", gTTS_lang='en')
        #self.invite_new_guest()
        #self.control(2, linear_x=0, angular_z=-1)


        textToSpeech("detect_old_guests_and_chair", gTTS_lang='en')
        self.detect_old_guests_and_chair()

        
        """
        self.Img_take_pictures_rosmsg.command = "take" 
        self.image_pub.publish(self.Img_take_pictures_rosmsg)
        
        self.detect_old_guests_and_chair()
        
        self.Img_take_pictures_rosmsg.command = "stop" 
        self.image_pub.publish(self.Img_take_pictures_rosmsg)
        """
        
    
       
                
                
                    

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





    def listing_recognition(self):

        # Define the microphone as the audio source
        microphone = sr.Microphone()

        # Create a recognizer object
        recognizer = sr.Recognizer()

        # Set the language for speech recognition
        language = 'en-US'  # Update with the desired language code

        spoken_texts_list = []

        # Function to process the audio data
        def process_audio():
            text = ""

            with microphone as source:
                print("Listening...")
                audio = recognizer.listen(source)

            try:
                # Perform speech recognition
                text = recognizer.recognize_google(audio, language=language)
                print("Recognized:", text)
            except sr.UnknownValueError:
                print("Could not understand audio")
            except sr.RequestError as e:
                print("Error:", str(e))

            return text

        # Continuously listen for audio input and process it
        with microphone as source:
            recognizer.adjust_for_ambient_noise(source)  # Adjust for ambient noise

        for i in range(3):
            try:
                text = process_audio()
                spoken_texts_list.append(text)
            except KeyboardInterrupt:
                print("Interrupted")
                break
        
        return spoken_texts_list




       #使用例
    def nlp_use(self, context):
        
        nlp = roberta_model_set()
        
        """
        QA_input = {
            'question': 'Why is model conversion important?',
            'context': 'The option to convert models between FARM and transformers gives freedom to the user and let people easily switch between frameworks.'
        }
        """

        QA_input1 = {
            'question': 'What is name?',
            'context': context
        }

        QA_input2 = {
            'question': 'What is favorite drink?',
            'context': context
        }

        QA_input_list = [QA_input1, QA_input2]

        #返答
        res_list =  roberta_model_res(nlp, QA_input_list)
        
        name = "unknown"
        drink = "unknown"
        
        print("res_list=" + str(res_list))  
        #print("len(res_list)=" + str())

        name = res_list[0]['answer']
        drink = res_list[1]['answer']

        name_score = res_list[0]['score']
        drink_score = res_list[1]['score']

        
        #名前と好きな飲み物を取得する   
        #print("name=" + name)
        #print("drink=" + drink)

        return name, drink, name_score, drink_score


    def audio_use(self):
        #文を喋らす
        #textToSpeech("I have a pen.", gTTS_lang='en')
        textToSpeech("What is your name and favorite drink.", gTTS_lang='en')
        
        #文を聞き取る．返り値が聞き取った文字列
        #res = recognize_speech(print_partial=True, remove_space=False, use_break=5, lang='en-us')
        res_list = self.listing_recognition()
        
        
        for res in res_list:

            if res != "":

                name, drink, name_score, drink_score = self.nlp_use(res)

                first_string = "Hi,Operater!" + name + "likes" + drink
                
                textToSpeech(first_string, gTTS_lang='en')
    


    def move_point(self):

        GOAL = [
            [1.3657, 0.0328, np.pi/2],
            [-0.4410, 0.0328, 0]
        ]

        p = PointToPoint()
        time.sleep(2)

        textToSpeech("目標1に行きます!")

        p.send_goal(GOAL[0])

        time.sleep(10)

        self.follow_new_guest()   

        #x座標を10等分
        """
        CUT_NUM = 10
        for i in range(CUT_NUM):
            pos_x = GOAL[0][0] * (i+1) / CUT_NUM
            pos_y = GOAL[0][1]
            pos_theta = GOAL[0][2]

            pos = [pos_x, pos_y, pos_theta]

            p.send_goal(pos)

            time.sleep(1)
        """



        while True:
            rospy.loginfo("1")
            if p.status == 3:
                break


        time.sleep(5)

        textToSpeech("目標2に行きます!")
        textToSpeech("ついてきてください。フォローミー")

        p.send_goal(GOAL[1])

        while True:
            rospy.loginfo("2")
            if p.status == 3:
                break

        """
        p.send_goal(GOAL[2])
        
        while True:
            rospy.loginfo("3")
            if p.status == 3:
                break
        """


    
        
if __name__ == "__main__":
    main_instance = Main()
    #main_instance.main_UDP()
    time.sleep(10)
    main_instance.move_point()

    #main_instance.main_ROS()
    
    #main_instance.audio_use()

    #spoken_texts_list = main_instance.listing_recognition()
    #print("spoken_texts_list=" + str(spoken_texts_list))