#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

from image_libs.memorize_img import forget_all_memory, memorize_chair_and_guest

from UDP_libs.UDP_client import UDP_Client
import time

class Image():
    def __init__(self):
        
        # udp
        self.SERVER_IP_ADRESS = "127.0.0.1"
        self.SERVER_PORT = 8890
        self.server_addr = (self.SERVER_IP_ADRESS, self.SERVER_PORT)
        
        self.UDP_CLIENT = UDP_Client(self.SERVER_IP_ADRESS, self.SERVER_PORT)

        
        self.STATE_LIST = ["新人ゲストの誘導", "椅子とゲストの確認"]
        self.TASK_LIST = ["開始", "終了"]
        self.state = self.STATE_LIST[0]

    #画像のメインルーチン
    def img_main(self):
        
        
        time.sleep(1)
        
        send_to_main_msgs = "img_init" #UDPの初期化
       
        self.UDP_CLIENT.send_message(send_to_main_msgs, self.server_addr)
        
        
        #全体ループ、(コードが一番下まで実行された場合でも上の状態へ戻れる)
        while True:
        
            #S_0
            if self.state == self.STATE_LIST[0]:
                
                """
                S_0時の処理
                """
                
                #次の状態に遷移するまで待つ
                while self.state != self.STATE_LIST[1]:
                    self.state = self.STATE_LIST[1]
                    
            
            
            
            #S_1
            if self.state == self.STATE_LIST[1]:
                
                """
                S_1時の処理
                """
                #recv_from_main_msgs = "S1_memorize"
                
                #次の状態に遷移するまで待つ
                while self.state != self.STATE_LIST[0]:
                    
                    recv_from_main_msgs, _ = self.UDP_CLIENT.get_message()
                    print("recv_from_main_msgs=" + str(recv_from_main_msgs))
                
                    if recv_from_main_msgs == "S1_memorize":
                        FOLDER = "/home/ri-one/catkin_ws/src/receptionist/src/short_memory/"
                        IMG = "chair_and_guest"
                        IMG_NUM = 5 #[枚]
                        TIME = 1 #[秒]  
                        
                        forget_all_memory(FOLDER, IMG)
                        memorize_chair_and_guest(FOLDER, IMG, IMG_NUM, TIME)
                    
                        self.state = self.STATE_LIST[0]
                        
                    
                exit()

if __name__ == "__main__":
    image = Image()
    image.img_main()
            