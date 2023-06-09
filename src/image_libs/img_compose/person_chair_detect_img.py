import torch
import cv2
import numpy as np

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
#椅子を中心座標が小さい順に並び替え、その要素を返す関数
def sort_chairs(value_list, idx_list):
  value_ndarray = np.array(value_list)
  value_argsorted_ndarray = np.argsort(value_ndarray)

  idx_sorted_list = []

  #要素番号を並び替える
  for i in range(len(idx_list)):
    idx_sorted_list.append(value_list[value_argsorted_ndarray[i]])
    
  return idx_sorted_list

#メインルーチン
def main():
  # Model
  #model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
  model = torch.hub.load('/home/ri-one/Desktop/github_local_repository/yolov5', 'custom', path='yolov5s.pt', source='local')

  #--- 検出の設定 ---
  PERSON_CLS = 0 #0:人
  CHAIR_CLS = 56 #56:椅子

  CONFIDENCE = 0.4  #信頼度を下げる (端の椅子が検出されない可能性がある)
  CLASSES = [PERSON_CLS, CHAIR_CLS] 

  PC_CAM_NUM = 0 #PC内蔵カメラのデバイス番号

  model.conf = CONFIDENCE 
  model.classes = CLASSES 

  #--- 映像の読込元指定 ---
  camera = cv2.VideoCapture(PC_CAM_NUM)              
  cap_count = 0 #1回だけ画像のサイズを取得する。

  #--- 画像のこの位置より左で検出したら、ヒットとするヒットエリアのためのパラメータ ---
  #pos_x = 240

  #人が写っていない前提で初期化する
  robo_p_dis = 3 #ロボットと人との距離感覚
  robo_p_drct = 3 #ロボットと人との方向感覚

  HEIGHT = 0 #カメラから取得した画像の高さを保持
  WIDTH = 0 #カメラから取得した画像の幅を保持


  """
  画像を読み込む
  """
  imgs = cv2.imread("stiched_img.png")

  HEIGHT, WIDTH = imgs.shape[:2]    




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

  print("chair_num=" + str(chair_num) +", person_num=" + str(person_num))


  """
  取得した視覚情報を処理する
  """
  #椅子が左、中、右のどこにあるかを判断するためのリスト
  chair_box_idx_list = []  #椅子の添字
  chair_box_cx_list = []   #椅子の中心座標
  person_box_idx_list = [] #人の添字
  person_box_cx_list = []  #人の中心座標

  idx = 0 #添字

  #複数の物体を検出する
  for *box, conf, cls in results.xyxy[0]:  # xyxy, confidence, class

      box_cx = int((int(box[0]) + int(box[2]))/2) #中心座標

      #椅子であった場合に、
      if int(cls) == CHAIR_CLS:

        #椅子の中心x座標を習得する
        chair_box_cx = box_cx
        chair_box_idx_list.append(idx)
        chair_box_cx_list.append(chair_box_cx)

      #人であった場合に、
      if int(cls) == PERSON_CLS:
        
        #椅子の中心x座標を習得する
        person_box_cx = box_cx 
        person_box_idx_list.append(idx)
        person_box_cx_list.append(person_box_cx)
        
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
      人が座っている椅子の数に応じて、椅子の方向の指し方を変える
      """
      chair_directions = ["left", "mid", "right"]
      #椅子が3つ検出された場合 (どの椅子にもゲストが座っていない)
      if chair_num == 3:
        chair_directions = ["left", "mid", "right"]

      #椅子が2つ検出された場合 (1つの椅子にもゲストが座っている)
      if chair_num == 2:
        chair_directions = ["left", "right"]

      #椅子が1つ検出された場合 (2つの椅子にもゲストが座っている)
      if chair_num == 1:

        if len(person_box_cx_list) != 0:

          #ロボットから見て右に椅子がある場合
          if person_box_cx_list[0] > WIDTH/2:
            chair_directions = ["right"]

          #ロボットから見て左に椅子がある場合
          elif person_box_cx_list[0] <= WIDTH/2:
            chair_directions = ["left"]
          
  cv2.imwrite("stiched_detect_img.png", imgs)  
      
  #--- 描画した画像を表示
  cv2.imshow('color',imgs)
  cv2.waitKey(1000) #[ms]
  cv2.destroyAllWindows()



  #以下のような矩形を追跡する(距離と位置で挙動を変える。)

  """
  if len(box_w_list) != 0:
    
    box_w_max = max(box_w_list) #最大となる矩形の幅を取得する
    box_w_max_idx = box_w_list.index(box_w_max) #最大となる矩形の添字を取得する
    box_cx = box_cx_list[box_w_max_idx] #その添字番目の矩形の中心のx座標を取得する

    if box_w_max < 350:
        #print(str(i) + "番目の人が遠い")
        robo_p_dis = 0 #ロボットは人が中央に来るまで前に進む

    elif box_w_max >= 350 and box_w_max <= 550:
        #print(str(i) + "番目の人が中央の距離")
        robo_p_dis = 1 #ロボットはそのまま

    elif box_w_max > 550:
        #print(str(i) + "番目の人が近い")
        robo_p_dis = 2 #ロボットは人が中央に来るまで後ろに下がる

    if box_cx < width/3:
        #print(str(i) + "番目の人が左にいる")
        robo_p_drct = 0 #ロボットは人が中央に来るまで左回りする

    elif box_cx > width/3 and box_cx < width * 2/3:
        #print(str(i) + "番目の人が中央の方向")
        robo_p_drct = 1 #ロボットはそのまま

    elif box_cx > width * 2/3:
        #print(str(i) + "番目の人が右にいる")
        robo_p_drct = 2 #ロボットは人が中央に来るまで右回りする
  """

  #print("追跡する矩形の添字番号=" + str(box_w_max_idx))
  #print("距離:" + str(robo_p_dis) + "、方向:" + str(robo_p_drct))

    


if __name__ == "__main__":
  main()