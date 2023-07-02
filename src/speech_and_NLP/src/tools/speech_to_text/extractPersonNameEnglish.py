import spacy

def extract_names(sentence):
    nlp = spacy.load("en_core_web_sm")
    doc = nlp(sentence)
    names = [entity.text for entity in doc.ents if entity.label_ == "PERSON"]
    return names