from speech_and_NLP.src.tools.speech_to_text.mecabAnalyzer import mecabAnalyzer


def katakana_to_hiragana(word):
    return word.translate(str.maketrans("カタカナ", "ひらがな"))


def is_meaning(text: str, word_list: list[str], path) -> bool:
    """
    日本語のテキストが、指定された単語リストの中に含まれる単語を持つかどうかを判定する関数

    Args:
        text (str): 判定したいテキスト
        word_list (list[str]): 判定する単語リスト。ひらがな、カタカナ、漢字の形で用意することを推奨。
        path (str, optional): mecabのpath。Defaults to "".

    Returns:
        bool: テキストが指定された単語リストの単語を持つ場合はTrue、それ以外はFalse。
    """
    mecab = mecabAnalyzer(path)
    words = mecab.parse(text).split()
    for word in words:
        analyzed_words = word.split(",")
        for analyzed_word in analyzed_words:
            hiragana_analyzed_word = katakana_to_hiragana(analyzed_word)
            for search in filter(None, word_list):
                if hiragana_analyzed_word == search:
                    return True
    return False


# text = "袋をとってきてください"
# verbs = ["ふくろ", "はこぶ","とる","もつ", "とって"]
# result = is_meaning(text, verbs, "-r /dev/null -d /usr/lib/mecab/dic/ipadic")
# print(result) # True
