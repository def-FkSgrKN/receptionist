# carry_my_luggage

## 環境構築

以下の環境で動くことを想定しています

- Turtlebot 2
- Rplidar A1
- Open Manipulator X
- Ubuntu 18.04
- ROS Melodic

### Rplidar A1

以下のパッケージのダウンロードとビルド

```
https://github.com/Slamtec/rplidar_ros
```

書き込み権限を与える(初めて使う場合)

```
ls /dev/ttyUSB*                 # どのデバイスファイルか確認
sudo chmod 666 /dev/ttyUSB0     # 実行権限を与える
```

### Open Manipulator X

[詳しくは公式ドキュメントを読もう](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/)

依存パッケージのインストール

```
sudo apt-get install ros-melodic-ros-controllers ros-melodic-gazebo* ros-melodic-moveit* ros-melodic-industrial-core
sudo apt-get install ros-melodic-dynamixel-sdk ros-melodic-dynamixel-workbench*
sudo apt-get install ros-melodic-robotis-manipulator
```

以下のパッケージのダウンロードとビルド

```
git clone https://github.com/ROBOTIS-GIT/open_manipulator.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
```

U2D2のセットアップ(初めて使う場合)

```
roscore
rosrun open_manipulator_controller create_udev_rules
```

### catkin_make の cmake エラー
```
rosdep install -r --from-path src --ignore-src
```

### speech_and_NLP

```
git submodule update --init
git submodule foreach git pull origin main
```
+ Ubuntuの場合 
```
cd speech_and_NLP
sh setup.sh
```
+ 手動でやる場合
```
pip install -r requirements.txt

or

pip3 ~~~ 
```

`cmake ffmpeg mecab libportaudio2 python-pyaudio` をインストール
```shell
sudo apt install cmake ffmpeg python-pyaudio mecab libportaudio2 -y
```

## 使い方

+ isMeaning
```python
rospy.wait_for_service("/isMeaning")
res = self.isMeaning("検出したい文章",["検出","したい","単語","をかく"])
res.res に booleanが返される
```
+ textToSpeech
```python
self.audio_pub.publish("発話させたい音声")
```
+ recognize_speech
```python
rospy.wait_for_service("/speechToText")
res = self.speechToText(中間テキスト表示非表示を設定(bool), 最低文字数, 名前のみ抽出するか(bool), 空白取り除くか(bool), voskLogLevel(-1でいいです))
res.resにテキストが入る
```

### フォルダ構成を以下のようにする
+ ROSから直接起動しないファイルをlibsフォルダに入れる
```
image_libs   画像のファイル群
audio_libs   音声のファイル群
control_libs 制御のファイル群
```
+ ROSから直接するファイルを外に置くと組み込むときにわかりやすい。以下は一例
```
image.py   視覚情報処理 
audio.py   聴覚情報処理、自然言語処理
control.py 運動制御処理
main.py    全体処理
```


### img_tasks内のファイルの詳細

+ Pythonファイル
```
glasses_detect.py   眼鏡の検出
pprbg_detect.py     紙袋の検出
person_detect_dd.py 人の検出(改良)
person_detect.py    人の検出(従来)
```

+ ptファイル(学習済モデルの重み)
```
yolov5s.pt 既存のyoloモデル  　             人の検出用
19sbest_glasses.pt 転移学習させたyoloモデル 眼鏡の検出用
22sbest_pprbg.pt   転移学習させたyoloモデル 紙袋用
```

+ 環境構築

```shell
yolov5のレポジトリを持ってくる
git clone https://github.com/ultralytics/yolov5.git
```

```python
model = torch.hub.load('yolov5の絶対Path(~/yolov5)', 'custom', path='22sbest_pprbg.pt', source='local')
```

