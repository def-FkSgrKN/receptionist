import nltk
from nltk.tokenize import word_tokenize, sent_tokenize
from nltk.tag import pos_tag
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
punkt_path = os.path.join(current_dir)

nltk.data.path.append(punkt_path)

def extract_favorite_drinks(text):
    drinks = []
    
    sentences = sent_tokenize(text)
    
    tagged_words = [pos_tag(word_tokenize(sentence)) for sentence in sentences]

    for sentence in tagged_words:
        for word, tag in sentence:
            if tag == 'NN' or tag.startswith('NN'):

                drinks.append(word.lower())
    
    return list(set(drinks))