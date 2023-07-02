from gtts import gTTS


def googleTextToMp3(text: str, path: str, lang: str = "ja"):
    """文字列をgTTSを通してMP3に変換する関数"""
    tts = gTTS(text, lang=lang, tld="com", slow=False)
    tts.save(path)
