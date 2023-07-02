import pyopenjtalk
import numpy as np
from scipy.io import wavfile


def textToWav(text: str, path: str, speed: float):
    """テキストから音声ファイルに変換する関数"""
    x, sr = pyopenjtalk.tts(text, speed)
    wavfile.write(path, sr, x.astype(np.int16))
