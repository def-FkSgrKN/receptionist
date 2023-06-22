from transformers import AutoModelForQuestionAnswering, AutoTokenizer, pipeline

model_name = "deepset/roberta-base-squad2"

# a) Get predictions
nlp = pipeline('question-answering', model=model_name, tokenizer=model_name)
"""
QA_input = {
    'question': 'Why is model conversion important?',
    'context': 'The option to convert models between FARM and transformers gives freedom to the user and let people easily switch between frameworks.'
}
"""

QA_input1 = {
    'question': 'What is his favorite drink?',
    'context': 'My name is Sogo Furukawa and My favorite drink is cola. Thank you!'
}

QA_input2 = {
    'question': 'What is his name?',
    'context': 'My name is Sogo Furukawa and My favorite drink is cola. Thank you!'
}

QA_input_list = [QA_input1, QA_input2]

for QA_input in QA_input_list:
    res = nlp(QA_input)
    print("res=" + str(res)) 
    



# b) Load model & tokenizer
#model = AutoModelForQuestionAnswering.from_pretrained(model_name)
#tokenizer = AutoTokenizer.from_pretrained(model_name)