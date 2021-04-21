import speech_recognition as sr

r = sr.Recognizer()
with sr.Microphone() as source: 
	print("Say something!")
	audio = r.listen(source, phrase_time_limit=10)

#recognize speech using Google Speech Recognition 
try:
	#for testing purposes we are just using the default API key 
	# to use another API key, use 'r.recognize_google(audio, key=:"GOOGLE_SPEECH_RECOGNITION_API')
	# instead of 'r.recognize_google(audio)'
	print("Google Speech thinks you said: "+ r.recognize_google(audio))
except sr.UnknownValueError:
	print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:
	print("Could not request results from Google Speech Recognition service; {0}".format(e))
	