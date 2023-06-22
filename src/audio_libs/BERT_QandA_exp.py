from transformers import AutoModelForQuestionAnswering, AutoTokenizer, pipeline

#https://huggingface.co/deepset/roberta-base-squad2
#モデルをセットする
def roberta_model_set():

    model_name = "deepset/roberta-base-squad2"

    # a) Get predictions
    nlp = pipeline('question-answering', model=model_name, tokenizer=model_name)
    
    return nlp


#質問に答える
def roberta_model_res(nlp, QA_input_list):

    res_list = []

    for QA_input in QA_input_list:
        res = nlp(QA_input)
        res_list.append(res)
        
    return res_list


#使用例
def example_use():
    
    nlp = roberta_model_set()
    
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

    #返答
    res_list =  roberta_model_res(nlp, QA_input_list)
    
    name = "unknown"
    drink = "unknown"
    
    print("res=" + str(res_list))  
    
    if res_list[0]['score'] > 0.5:
        drink = res_list[0]['answer']
        
    if res_list[1]['score'] > 0.5:
        name = res_list[1]['answer']
    
    #名前と好きな飲み物を取得する   
    print("name=" + name)
    print("drink=" + drink)
       

# b) Load model & tokenizer
#model = AutoModelForQuestionAnswering.from_pretrained(model_name)
#tokenizer = AutoTokenizer.from_pretrained(model_name)

if __name__ == "__main__":
    example_use()