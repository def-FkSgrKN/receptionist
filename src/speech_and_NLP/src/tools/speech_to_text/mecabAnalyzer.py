import MeCab


def mecabAnalyzer(model_path):
    if model_path == None:  # NOQA
        return MeCab.Tagger()
    return MeCab.Tagger(model_path)
