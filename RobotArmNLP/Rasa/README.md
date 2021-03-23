
## Useful RASA commands to remember 
1) ``` rasa train nlu --nlu data/train.yml```(To train the nlu model)
2) ```rasa shell``` 
3) ```rasa test nlu --nlu tests/test.yml``` ([Testing Your Assistant (rasa.com)](https://rasa.com/docs/rasa/testing-your-assistant/))
	This outputs the data in a results folder. Important to metric to look at is the confusion matrix to see where its messing up. 

## Possible Classifiers we can use
Useful Resources: https://rasa.com/docs/rasa/components
 
### Intent Classifiers
1) MitieIntentClassifier: using a multi-class linear SVM with a sparse linear kernel 
2) SklearnIntentClassifier: similar to Mitie
3) KeywordIntentClassifier: 
	- works by searching intended for small, short-term projects
	- ***Could be interesting to test***
	- The model is case-sensitive
4) DIETClassifier: 
	- Does intent recognition and entity recognition together 
	- Based on a transformer architecture (has been pretty successful for more general NLP) 
	- ***Could be interesting to test***
	- Optimizing this classifier: https://botfront.io/blog/better-intent-classification-and-entity-extraction-with-diet-classifier-pipeline-optimization

### Entity Extractor 
1) MitieEntityExtractor
2) SpacyEntityExtractor
	- Seems kind of rigid 
	- Does not give confidence scores (might not be a problem?) 
	- ***Could be interesting to test***
3) CRFEntityExtractor
	- Seems modular 
	- I personally really like CRF vs DietClassifier (Sneha) 
	- ***Could be interesting to test***
4) DucklingHTTPExtractor
	- Seems good for simple things like name, places 
	- our use case might be too niche 
5) DIETClassifier
6) RegexEntityExtractor
	- Uses lookup tables: We might like this because our usecase is so restricted 
7) EntitySynonymMapper
	- Doesn't seem to be useful for picking up entities but can be used to refine the entities we picked up 
