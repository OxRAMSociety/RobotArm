from rasa.nlu.model import Interpreter
interpreter = Interpreter.load("../models/nlu") #Unzip the model and write the path to that model
my_prediction = interpreter.parse("hi")
print(my_prediction)
