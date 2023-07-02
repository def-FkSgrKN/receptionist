import Levenshtein

def find_nearest_word(input_str, word_list):
    """
    引数:
        input_str (str): 検索したい文字列
        word_list (list): 検索対象の単語リスト
    返り値:
        str: 検索した文字列に一番近い単語
    説明:
        検索したい文字列に一番近い単語を検索対象の単語リストから見つけて返します。
        Levenshtein距離を使用して、文字列と各単語の距離を計算します。
        計算した距離が最も小さい単語を見つけ、その単語を返します。
    """
    min_distance = float('inf')
    nearest_word = None
    
    for word in word_list:
        distance = Levenshtein.distance(input_str, word)
        if distance < min_distance:
            min_distance = distance
            nearest_word = word
    
    return nearest_word