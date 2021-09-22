from flask import Flask,render_template,url_for,request
from flask_cors import CORS, cross_origin
import speech_recognition as sr
from rasa.nlu.model import Interpreter


app = Flask(__name__)
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'


interpreter = None 
def load_model():
	global interpreter 
	interpreter = Interpreter.load("../Rasa/models/nlu") #Unzip the model and write the path to that model

@app.route('/')
@cross_origin()
def home():
	#rendering an outdated html template
	return render_template('home.html')

@app.route('/record')
@cross_origin()
def record():
	r = sr.Recognizer()
	with sr.Microphone() as source: 
		audio = r.listen(source, phrase_time_limit=5)
	try: 
		message = (r.recognize_google(audio))
	except sr.UnknownValueError:
		message = "Did not understand"
	
	# rasa_data = interpreter.parse(message)
	# my_prediction = ["intent: "+rasa_data['intent']['name']] + [rasa_data['entities'][i]['entity']+": "+rasa_data['entities'][i]['value'] for i in range(len(rasa_data['entities']))]

	# result= {"message": message, "prediction": my_prediction}
	result={"message": message}
	return result

@app.route('/predict')
@cross_origin()
def predict():
	message = "null"
	message = request.args["message"]
	rasa_data = interpreter.parse(message)
	prediction = {"prediction": ["intent: "+rasa_data['intent']['name']] + [rasa_data['entities'][i]['entity']+": "+rasa_data['entities'][i]['value'] for i in range(len(rasa_data['entities']))]}
	return prediction



if __name__ == '__main__':
	app.debug = True
	print("Loading model and flask starting server...")
	load_model() #Preloading the model 
	print("Model is loaded")
	app.run(debug=True)