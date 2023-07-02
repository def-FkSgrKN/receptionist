from speech_and_NLP.src.tools.text_to_speech.textToWav import textToWav
from speech_and_NLP.src.tools.text_to_speech.getAudioLength import getAudioLength
from speech_and_NLP.src.tools.text_to_speech.playAudio import playAudio
from speech_and_NLP.src.tools.text_to_speech.googleTextToMp3 import googleTextToMp3
from speech_and_NLP.src.tools.text_to_speech.englishToWav import englishToWav
import os


def textToSpeech(
    text: str,
    speed: float = 1,
    path: str = "audio.wav",
    remove: bool = True,
    gTTS_lang: str = None,
    isEnglish: bool = False,
):
    """
    文字列を与えると音声を再生する関数

    Args:
        text (str): 発話させるテキスト
        speed (float, optional): 音声の再生速度. 1で標準速度, 0.5で1/2の速度. Defaults to 1.
        path (str, optional): 音声を保存させるパス. Defaults to "audio.wav".
        remove (bool, optional): 音声ファイルを削除するかどうか. Defaults to True.
        gTTS_lang (str, optional): gTTSの言語を文字列で言語を指定。指定しない場合はpyopenjtalkの使用 Defaults to None (インターネット接続が必要です。).
        isEnglish (bool, optional): englishToWav関数を使うかどうか。Defaults to False.

    Returns:
        None
    """

    if gTTS_lang:
        path = f"{os.path.splitext(path)[0]}.mp3"
        try:
            googleTextToMp3(text, path, gTTS_lang)
        except:  # NOQA
            englishToWav(text, path) if isEnglish else textToWav(text, path, speed)
    else:
        englishToWav(text, path) if isEnglish else textToWav(text, path, speed)

    playAudio(path, getAudioLength(path))

    if remove:
        os.remove(path)
