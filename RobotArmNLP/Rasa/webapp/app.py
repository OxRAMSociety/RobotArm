from flask import Flask,render_template,url_for,request
#import pandas as pd 
#import pickle
#from sklearn.feature_extraction.text import CountVectorizer
#from sklearn.naive_bayes import MultinomialNB
#from sklearn.externals import joblib
from rasa.nlu.model import Interpreter
#import tensorflow as tf

app = Flask(__name__)


interpreter = None 
def load_model():
	global interpreter 
	interpreter = Interpreter.load("../models/nlu") #Unzip the model and write the path to that model


@app.route('/')
def home():
	return render_template('home.html')

@app.route('/predict',methods=['POST'])
def predict():
	
	#interpreter = Interpreter.load("../models/nlu") #Unzip the model and write the path to that model

	if request.method == 'POST':
		message = request.form['message']
		rasa_data = interpreter.parse(message)
		my_prediction = ["intent: "+rasa_data['intent']['name']] + [rasa_data['entities'][i]['entity']+": "+rasa_data['entities'][i]['value'] for i in range(len(rasa_data['entities']))]
	return render_template('result.html',prediction = my_prediction)



if __name__ == '__main__':
	app.debug = True
	print("Loading model and flask starting server...")
	load_model() #Preloading the model 
	print("Model is loaded")
	app.run(debug=True)