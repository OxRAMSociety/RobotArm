from flask import Flask,render_template,url_for,request
import speech_recognition as sr


app = Flask(__name__)

@app.route('/')
def home():
	return render_template('home.html')

@app.route('/record')
def record():
	r = sr.Recognizer()
	with sr.Microphone() as source: 
		audio = r.listen(source, phrase_time_limit=10)
	try: 
		text = (r.recognize_google(audio))
	except sr.UnknownValueError:
		text = "Did not understand"
	print(text)
	return render_template('result.html', command = text)






if __name__ == '__main__':
	app.debug = True
	app.run(debug=True)