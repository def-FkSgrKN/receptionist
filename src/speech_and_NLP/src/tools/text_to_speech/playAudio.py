from os import environ

environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"  # NOQA
import pygame  # NOQA
import time  # NOQA


def playAudio(path: str, playTime: int):
    """音声ファイルを再生する関数"""
    pygame.mixer.init()
    pygame.mixer.music.load(path)
    pygame.mixer.music.play()
    time.sleep(playTime)
    pygame.mixer.music.stop()
