import vosk
import json
from speech_and_NLP.src.tools.speech_to_text.extractPersonName import extractPersonName
import pyaudio


def recognize_speech(
    print_partial: bool = True,
    use_break: bool or int = True,
    return_extract_person_name: bool or str = False,
    remove_space: bool = True,
    voskLogLevel: int = -1,
    path="",
    lang: str = "ja",
) -> str:
    """
    録音した音声を文字起こしする関数
    * 大会本番でエラーにならないようオフライン環境で必ず一度実行してください。初回利用時にモデルをダウンロードします。

    Args:
        print_partial (bool): 中間テキストを表示するかどうかを設定します。

        use_break (int): 最終テキスト取得後、関数から抜ける文字数を指定します。

        return_extract_person_name (str): 名前のみ抽出を行う場合、"array" と指定します。
        "array" を指定すると、戻り値の1つ目に名前、2つ目に最終テキストが入った配列が返されます。

        remove_space (bool): 空白を取り除くかどうかを指定します。

        path (str): mecabのpathを指定します。

        lang (str): voskのmodelを指定します。


    Returns:
        string: 文字起こしされた文字列
    """

    print("VOSK LOADING .....")
    vosk.SetLogLevel(voskLogLevel)
    model = vosk.Model(lang=lang)

    p = pyaudio.PyAudio()
    stream = p.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        input=True,
        frames_per_buffer=8000,
    )

    recognizer = vosk.KaldiRecognizer(model, 16000)

    text = ""

    while True:
        data = stream.read(8000)
        if len(data) == 0:
            break
        if recognizer.AcceptWaveform(data):
            result = recognizer.Result()
            result_dict = json.loads(result)["text"]
            if remove_space:
                result_dict = result_dict.replace(" ", "")
            text += result_dict
            print(result_dict)
            if isinstance(use_break, bool) and use_break:
                break
            elif isinstance(use_break, (int, float)):
                if len(result_dict) > use_break:
                    break
        else:
            result = recognizer.PartialResult()
            result_dict = json.loads(result)["partial"]
            if remove_space:
                result_dict = result_dict.replace(" ", "")
            if print_partial:
                print(result_dict)

    if return_extract_person_name == "array":
        return [extractPersonName(text, path), text]
    elif return_extract_person_name:
        return extractPersonName(text, path)
    return text
