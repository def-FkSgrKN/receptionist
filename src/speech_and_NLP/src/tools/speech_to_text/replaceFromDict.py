import re
from typing import Dict

import json

DICTIONARY_PATH = "speech_and_NLP/src/config/dictionary.json"

dictionary = json.load(open(DICTIONARY_PATH))


def replaces(text: str, trdict: Dict[str, str] = dictionary) -> str:
    """
    与えられた文字列に対して、辞書に従って置換を行う関数(手動スペル訂正機能)
    :param text: 置換を行う対象の文字列
    :param trdict: 辞書。keyに置換前の文字列、valueに置換後の文字列を指定する。
    :return: 置換が行われた文字列

    "speech_and_NLP/src/config/dictionary.json"に左側に置換前、右側に置換後を書くと自動で変換されます。
    """
    return re.sub(
        "|".join(trdict.keys()),
        lambda m: next(
            (
                re.sub(pattern, trdict[pattern], m.group(0))
                for pattern in trdict
                if re.fullmatch(pattern, m.group(0))
            )
        ),
        text,
    )
