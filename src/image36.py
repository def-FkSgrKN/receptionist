#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

"""
(image36_ros.py:5979): Gtk-ERROR **: 12:06:10.745: GTK+ 2.x symbols detected. Using GTK+ 2.x and GTK+ 3 in the same process is not supported
Trace/breakpoint trap (コアダンプ)


Gtk3では、動かなかったため

OpenCVに必要なpkg Gtkのバージョンを選択している。
"""
import gi
gi.require_version('Gtk', '2.0')
from gi.repository import Gtk

import rospy
import torch
import cv2
import numpy as np
import time
import os

from receptionist.msg import Img_detect, Img_take_pictures
from image_libs.panolama_img import img_compose

"""
[方法1]

信頼度50％で、人が椅子に座っているときに椅子が検出されにくいことを利用し、
椅子の数を数える。

実際の椅子の数が3つのとき、
検出された椅子の数が2つしかないとき、その場所には人が座っていると推定する
左:椅子、中:椅子、右:椅子 => 左:椅子、中:人、右:椅子


[方法2]

信頼度30％で、人と重なる椅子の個数を数える

実際の椅子の数が3つのとき、
検出された椅子の数が3つで、人の数が1つのときその場所には人が座っていると推定する
左:椅子、中:椅子、右:椅子 => 左:椅子、中:椅子と人、右:椅子
"""

"""
model.classes = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 
5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 
10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 
14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 
20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack', 
25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee', 
30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat', 
35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 
39: 'bottle', 40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 
45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 
51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 
58: 'potted plant', 59: 'bed', 60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 
64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven', 
70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase', 
76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'}
"""

class Image36():
  
  def __init__(self):
    #node
    rospy.init_node("image36")
    
    #msg
    self.Img_detect_rosmsg = Img_detect()
    
    #publisher
    self.image_pub = rospy.Publisher("/image_detect", Img_detect, queue_size=1)
    
    #subscriber
    rospy.Subscriber("/image_take_pictures", Img_take_pictures, self.image_take_pictures_callback)
    self.img_take_pictures_command = ""
    
    #camera
    self.PC_CAM_NUM = 0 #PC内蔵カメラのデバイス番号
    self.camera = cv2.VideoCapture(self.PC_CAM_NUM)        
    
    self.PERSON_CLS_STR = "person"
    self.CHAIR_CLS_STR = "chair"
    
    #take pictures
    self.img_count = 0 #撮影枚数
    self.FOLDER = "/home/ri-one/catkin_ws/src/receptionist/src/short_memory/"
    self.IMG = "chair_and_guest"
    
    self.compose_count = 0 #合成回数


  #callback関数
  def image_take_pictures_callback(self, msg):
      self.img_take_pictures_command = msg.command
    

  #椅子を中心座標が小さい順に並び替え、その要素を返す関数
  def sort_chairs(self, value_list, idx_list):
    value_ndarray = np.array(value_list)
    value_argsorted_ndarray = np.argsort(value_ndarray)

    idx_sorted_list = []

    #要素番号を並び替える
    for i in range(len(idx_list)):
      idx_sorted_list.append(value_list[value_argsorted_ndarray[i]])
      
    return idx_sorted_list


  #特定のフォルダ内の写真をすべて削除する
  def forget_all_memory(self, FOLDER, IMG):

      count = 1

      while True:
          
          img_path = FOLDER + IMG + str(count) + ".png"
          count += 1

          #画像を読む
          if os.path.isfile(img_path):
              os.remove(img_path)
              
          #存在しなければ抜ける
          else:
              break


  #メインルーチン
  def main(self):
    # Model
    #model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    
    YOLO_PATH = "/home/ri-one/yolov5/" #大会用PC
    #YOLO_PATH = "/home/ri-one/Desktop/github_local_repository/yolov5" #個人用PC
    
    MODEL_PATH = "/home/ri-one/catkin_ws/src/receptionist/src/image_libs/yolov5s.pt" #大会用 launch起動時
    #MODEL_PATH = "yolov5s.pt" #個人用 python起動時
    
    model = torch.hub.load(YOLO_PATH, 'custom', path=MODEL_PATH, source='local')

    #--- 検出の設定 ---
    PERSON_CLS = 0 #0:人
    CHAIR_CLS = 56 #56:椅子

    CONFIDENCE = 0.5  #信頼度
    CLASSES = [PERSON_CLS, CHAIR_CLS] 

    model.conf = CONFIDENCE 
    model.classes = CLASSES 

        
    cap_count = 0 #1回だけ画像のサイズを取得する。
    
    change_imgs = 0

    #--- 画像のこの位置より左で検出したら、ヒットとするヒットエリアのためのパラメータ ---
    #pos_x = 240

    #人が写っていない前提で初期化する

    HEIGHT = 0 #カメラから取得した画像の高さを保持
    WIDTH = 0 #カメラから取得した画像の幅を保持

    time.sleep(1)

    take_pictures_start_time = time.time()
    take_pictures_end_time = 0
    TAKE_PICTURES_DELTA_TIME = 1 #[s]



    while True:

      """
      画像をカメラから取得する
      """
      ret, imgs = self.camera.read() #映像から１フレームを画像として取得

      if cap_count == 0:
        HEIGHT, WIDTH = imgs.shape[:2]    
        print("幅:" + str(WIDTH) + "、高さ:" + str(HEIGHT))
        cap_count = 1 
        
        
      """
      主部からの指示で写真を撮影する
      """      
      if self.img_take_pictures_command == "take":
        
        take_pictures_end_time = time.time()
        
        #前のデータを削除する。
        if self.img_count == 0:
          self.forget_all_memory(self.FOLDER, self.IMG)
          self.img_count += 1
          self.compose_count = 0
          
          
        #一定時間経過後写真を撮る
        elif take_pictures_end_time - take_pictures_start_time > TAKE_PICTURES_DELTA_TIME:
          
          take_pictures_start_time = take_pictures_end_time
          
          img_path = self.FOLDER + self.IMG + str(self.img_count) + ".png" #画像の保存場所
          cv2.imwrite(img_path, imgs) # 画像を保存する
          self.img_count += 1


      #画像を合成し、検出する
      elif self.img_take_pictures_command == "stop" and self.compose_count == 0:
        
        composed = img_compose(self.FOLDER, self.IMG)
        results = model(composed, size=160) 
        
        for *box, conf, cls in results.xyxy[0]:  # xyxy, confidence, class

          box_c_x = int((int(box[0]) + int(box[2]))/2) #中心x座標
          box_c_y = int((int(box[1]) + int(box[3]))/2) #中心y座標
          width = int(box[2] - box[0]) #幅
          height = int(box[3] - box[1]) #高さ　
          
          x_mid_list.append(box_c_x)
          y_mid_list.append(box_c_y)
          width_list.append(width)
          height_list.append(height)
          
          #枠描画
          cv2.rectangle(
              composed,
              (int(box[0]), int(box[1])), #xmin, ymin
              (int(box[2]), int(box[3])), #xmax, ymax
              color=frame_color, #枠の色
              thickness=2, #幅2
              )
          
        cv2.imshow("composed", composed)
        
        self.compose_count = 1
        
        

      """
      モデルから結果を取得する

      print("results.xyxy[0]=" + str(results.xyxy[0]))

      0:xmin、1:ymin、2:xmax、3:ymax、4:信頼度、5クラス
      results.xyxy[0]=tensor([[ 91.77512,   6.83427, 638.87476, 470.62402,   0.78496,   0.00000]])  
      """

      results = model(imgs, size=160) #--- 160ピクセルの画像にして処理


      """
      視覚から得た、物体の個数を数える
      """
      chair_num = 0  #写った椅子の個数
      person_num = 0 #写った人の個数

      for *box, conf, cls in results.xyxy[0]:  # xyxy, confidence, class
        
        #各クラスごとに個数を数える
        if int(cls) == CHAIR_CLS:
            chair_num += 1

        elif int(cls) == PERSON_CLS:
            person_num += 1
      
      #print("chair_num=" + str(chair_num) +", person_num=" + str(person_num))


      """
      取得した視覚情報を送る
      
      uint16 count
      string[] class_list
      uint16[] x_mid_list
      uint16[] y_mid_list
      uint16[] width_list
      uint16[] height_list
      """
      
      #椅子が左、中、右のどこにあるかを判断するためのリスト
      chair_box_idx_list = []  #椅子の添字
      chair_box_cx_list = []   #椅子の中心座標
      person_box_idx_list = [] #人の添字
      person_box_cx_list = []  #人の中心座標

      idx = 0 #添字
      
      count = len(results.xyxy[0])
      class_list = []
      x_mid_list = []
      y_mid_list = []
      width_list = []
      height_list = []
      
      for *box, conf, cls in results.xyxy[0]:  # xyxy, confidence, class

          box_c_x = int((int(box[0]) + int(box[2]))/2) #中心x座標
          box_c_y = int((int(box[1]) + int(box[3]))/2) #中心y座標
          width = int(box[2] - box[0]) #幅
          height = int(box[3] - box[1]) #高さ　
          
          x_mid_list.append(box_c_x)
          y_mid_list.append(box_c_y)
          width_list.append(width)
          height_list.append(height)
          
          

          #椅子であった場合に、
          if int(cls) == CHAIR_CLS:
      
            #椅子の中心x座標を習得する
            """
            chair_box_cx = box_cx
            chair_box_idx_list.append(idx)
            chair_box_cx_list.append(chair_box_cx)
            """
            class_list.append(self.CHAIR_CLS_STR)

          #人であった場合に、
          if int(cls) == PERSON_CLS:
            
            #椅子の中心x座標を習得する
            """
            person_box_cx = box_cx 
            person_box_idx_list.append(idx)
            person_box_cx_list.append(person_box_cx)
            """
            class_list.append(self.PERSON_CLS_STR)
            
          idx += 1 #添字を加算する

          #枠色 と 文字色 の指定
          frame_color = (255,0,0)        #枠用  :青色
          text_color = (255, 255, 255) #文字用:白色

          #枠描画
          cv2.rectangle(
              imgs,
              (int(box[0]), int(box[1])), #xmin, ymin
              (int(box[2]), int(box[3])), #xmax, ymax
              color=frame_color, #枠の色
              thickness=2, #幅2
              )

          #クラス名と信頼度を文字列変数に代入
          s = model.names[int(cls)]+":"+'{:.1f}'.format(float(conf)*100)

          #文字枠と文字列描画
          #yoloの中よりも自分で描画した方が非常に高速
          cv2.rectangle(imgs, (int(box[0]), int(box[1])-20), (int(box[0])+len(s)*10, int(box[1])), frame_color, -1)
          cv2.putText(imgs, s, (int(box[0]), int(box[1])-5), cv2.FONT_HERSHEY_PLAIN, 1, text_color, 1, cv2.LINE_AA)

          #追いかけるマーカーを描画
          cv2.circle(imgs, (int((box[0]+box[2])/2), int((box[1]+box[3])/2)), 15, (255, 255, 255), thickness=-1)

          #angle = int((width - (box[0]+box[2])/2)/width * 180)
      
      """
      print("\n")
      print("count=" + str(count))
      print("class_list=" + str(class_list))
      print("x_mid_list=" + str(x_mid_list))
      print("y_mid_list=" + str(y_mid_list))
      print("width_list=" + str(width_list))
      print("height_list=" + str(height_list))
      """
      
      """
      主部へ送るためにデータを梱包する
      """
      self.Img_detect_rosmsg.count = count
      self.Img_detect_rosmsg.class_list = class_list
      self.Img_detect_rosmsg.x_mid_list = x_mid_list
      self.Img_detect_rosmsg.y_mid_list = y_mid_list
      self.Img_detect_rosmsg.width_list = width_list
      self.Img_detect_rosmsg.height_list = height_list

      self.image_pub.publish(self.Img_detect_rosmsg) #出版


      #print("self.img_take_pictures_command=" + str(self.img_take_pictures_command))
     
      

          
          
      
      
      #print("self.Img_detect_rosmsg.x_mid_list=" + str(self.Img_detect_rosmsg.x_mid_list))
      
      
      

      #print("追跡する矩形の添字番号=" + str(box_w_max_idx))
      #print("距離:" + str(robo_p_dis) + "、方向:" + str(robo_p_drct))

      #--- 描画した画像を表示
      cv2.imshow('color',imgs)

      #「q」キー操作があればwhileループを抜ける
      if cv2.waitKey(1) & 0xFF == ord('q'):
        break


#外部呼び出し用 class外関数
def get_image_size(PC_CAM_NUM=0):    
  
    camera = cv2.VideoCapture(PC_CAM_NUM)     
    ret, imgs = camera.read() #映像から１フレームを画像として取得
    HEIGHT, WIDTH = imgs.shape[:2]    
    return HEIGHT, WIDTH


if __name__ == "__main__":
  
  image36_instance = Image36()
  image36_instance.main() 