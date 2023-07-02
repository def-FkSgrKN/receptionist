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


