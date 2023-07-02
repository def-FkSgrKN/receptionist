import subprocess


def englishToWav(text: str, save_path: str):
    """
    espeakを使用して、テキストを音声ファイルに変換する関数

    Args:
        text (str): 変換したい文字列
        save_path (str): 保存するファイルパス

    Returns:
        None
    """
    command = ["espeak", "-w", save_path, text]

    subprocess.run(command)
