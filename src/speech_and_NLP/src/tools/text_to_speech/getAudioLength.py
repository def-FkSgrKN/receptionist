from pydub import AudioSegment


def getAudioLength(path: str) -> float:
    """音声ファイルの長さを取得する関数。

    Args:
        path (str): 音声ファイルのパス。

    Returns:
        float: 音声ファイルの長さ（秒）。
    """
    sound = AudioSegment.from_file(path)
    duration = sound.duration_seconds
    return duration
