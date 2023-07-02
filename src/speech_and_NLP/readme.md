# speech_and_NLP

## 機能一覧
+ 発話(オフラインだと日本語と英語、オンラインだとgTTSの対応する範囲)
+ openjtalk再生速度調整
+ VOSKによる文字起こし(対応言語多数)
+ mecabによる形態要素解析
+ 人名抽出、単語検索(日本語)
+ JSONファイルからの置換機能
+ Levenshteinを用いた近い単語の抽出
+ 音声ファイル再生、音声ファイルの再生時間の取得
+ スペルミス修正(英語のみ)

## 動作確認環境

* Python 3.6 以上

(一部のコミットでは3.10を必要とします。)

# インストール
`setup.sh`を実行する。

`pip install -r requirments.txt`を実行する。

* 手動でのインストール
`cmake ffmpeg mecab libportaudio2 espeak`が必要です。aptもしくはapt-getでインストールしてください。

```sh
python -m spacy download en
```
も実行してください。

その他pipで必要なライブラリ


```
autocorrect
unidic 
vosk
sounddevice
mecab-python3
pygame --pre
mutagen
gTTS==2.2.3
pyopenjtalk
pydub
scipy
nltk
spacy
```

また
```
docker-compose up --build
```
はマイク等のテストは行っていません。

## 使い方
関数の上をホバーすれば説明が出ます。
(VSCode)


example.pyをここに掲載します。
```Python
# importはあくまで例です。パスは適宜変更してください。以下のimportも同様です。
from speech_and_NLP.src.speechToText import recognize_speech
from speech_and_NLP.src.textToSpeech import textToSpeech

#
##
### 主な関数の使い方は関数をホバーすると表示されます。引数や返り値の説明も表示されます。
##
#


# 最低限の使い方

textToSpeech("こんにちは")
text = recognize_speech()




# textToSpeech() は、音声合成を行う関数です。

textToSpeech(text="話す文字列", speed="ここにfloat型で速度を設定します。Default: 1.0", path="ここに音声ファイルの保存先を設定します。Default: audio.wav", remove="ここに音声ファイルを削除するかどうかを設定します。Default: True", gTTS_lang="ここに音声合成の言語を設定します。ただし、インターネット環境が必要です。Default: ja", isEnglish="ここに英語を認識するかどうかを設定します。espeakを使用します。Default: False")

# recognize_speech() は、音声認識を行う関数です。返り値は音声認識した文字列でstr型です。

text = recognize_speech(print_partial="中間テキストを表示するかどうか設定します。Default: True", use_break="音声認識を終了するかどうか設定します。int型で数値を渡すとその文字数が来るとbreakします。Default: False", return_extract_person_name="音声認識したテキストから人名を抽出するかどうか設定します。Default: False", remove_space="音声認識したテキストから空白を削除するかどうか設定します。Default: True", voskLogLevel="voskのログレベルを設定します。Default: 0", path="mecabの辞書のパスを設定します。Default: ''", lang="音声認識の言語を設定します。Default: 'ja'")

# その他の使い方

# speech_to_text編

from speech_and_NLP.src.tools.speech_to_text.isMeaning import is_meaning
# 以下の関数は、音声認識したテキストに含まれる単語が、音声認識したテキストに含まれる単語のリストに含まれているかどうかを判定します。
# この関数の返り値はbool型です。
is_meaning(text="テキスト", word_list="音声認識したテキストに含まれる単語のリスト", path="mecabの辞書のパスを設定します。Default: ''")

from speech_and_NLP.src.tools.speech_to_text.extractPersonName import extractPersonName
# この関数の返り値は音声認識したテキストに含まれる人名でstr型です。
extractPersonName(text="テキスト", path="mecabの辞書のパスを設定します。Default: ''")

from speech_and_NLP.src.tools.speech_to_text.replaceFromDict import replaces
# この関数の返り値は置換したテキストでstr型です。
replaces(text="テキスト", trdict="置換する辞書")

from speech_and_NLP.src.tools.speech_to_text.spellcheck import correctSpell
# この関数の返り値はスペルチェックしたテキストでstr型です。
correctSpell(text="テキスト") # ただし英語のみ対応しています。

from speech_and_NLP.src.tools.speech_to_text.findNearstWord import find_nearest_word
# この関数は検索したい文字列に一番近い単語を検索対象の単語リストから見つけて返します。Levenshtein距離を使用して、文字列と各単語の距離を計算します。計算した距離が最も小さい単語を見つけ、その単語を返します。str型です。
find_nearest_word(text="テキスト", word_list="単語のリスト")

# text_to_speech編

from speech_and_NLP.src.tools.text_to_speech.getAudioLength import getAudioLength
# この関数の返り値は音声ファイルの長さでfloat型です。
getAudioLength(path="音声ファイルのパス")

from speech_and_NLP.src.tools.text_to_speech.playAudio import playAudio
# この関数は音声ファイルを再生するだけです。
playAudio(path="音声ファイルのパス")

from speech_and_NLP.src.tools.text_to_speech.textToWav import textToWav
# この関数は音声ファイルを保存するだけです。
textToWav(text="テキスト", path="音声ファイルのパス", speed="ここにfloat型で速度を設定します。Default: 1.0")
```

## 使用技術
textToSpeech ↓

`pyopenjtalk, espeak, gTTS, pygame, pydub`

pyopenjtalkはオフラインで日本語の発話を行っています。

espeakはオフラインで英語の発話を行っています。

gTTSはオンラインでの発話を行っています。

pygameは音声ファイルの再生に使用しています。

pydubは音声ファイルの長さを取得するのに使用しています。

speechToText ↓

`VOSK, mecab, Levenshtein, speller`

VOSKはオフラインでの文字起こしを行っいます。

mecabはオフラインで日本語の解析に使用しています。mecabを使用することで形態要素解析を行うことができ、動詞や人名の抽出を行います。

Levenshteinは単語の距離を測るライブラリで最も近い意味の単語を探す関数に使用しています。

spellerは英語のスペルを修正する関数に使用しています。


## 想定している運用方法


オフライン限定の場合はpyopenjtalk及びespeakを使用して発話をする。

VOSKで聞き取る。

VOSKの言語モデルは引数で渡せるようにしているので適宜英語などに変更してください。

gTTSはオンラインで効果を発揮しますが、gTTSは音声ファイルの作成を行うのであらかじめ決められた音声のみを発話する場合や種類が少ない場合はあらかじめ用意しておくことで高音質の音声を再生することができます。
またgTTSはかなりの言語に対応しています。

mecabで動詞などの抽出をして想定した動詞があるか検索したりします。

Levenshteinで近い単語を探し出すこともできます。

## トラブルシューティングなど


mecabの辞書がみつからないとき

recognize_speech()関数のpath引数に適切なmecabの辞書のパスを当ててください。

その他リンクを下に貼ってまるので参考にしてください。

### コードの改変など

src/tools以下のspeech_to_text及び、text_to_speechに関数などを作成して使いまわすと良いと思います。

#### その他

Python2のROS1環境ではPython3を必要とするこのプログラムは動作しません。

そのため、publish, subscriberやserviceなどで通信して値の受け渡しをするほうが良いと思います。(carry_my_luggage find_my_matesは実装済み)

またはROS1 + Python3の環境もしくはROS2の場合はfrom importでそのまま使用できるはずです。(未検証)

ROS1 Python3の環境構築は
```
sudo apt-get install ros-melodic-ros-core -y
sudo apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
echo "alias python='python3'" >> ~/.bashrc
echo "alias python='python3'" >> ~/.zshrc
```

ROS2の環境構築は


https://github.com/ryuichiueda/ros2_setup_scripts.git


など


もしくは

git@github.com:rionehome/ros_galactic_setup.git

git@github.com:rionehome/ros_humble_setup.git

を使用すると良いと思います。

### ブランチ戦略
+ GitHub Flow


## リンク
* [ArchLinuxにMecabをインストール](https://www.komee.org/entry/2018/02/28/120128)
* [MeCab](https://taku910.github.io/mecab/)
* [VOSK](https://alphacephei.com/vosk/)
* [SoundDevice](https://pypi.org/project/sounddevice/)
