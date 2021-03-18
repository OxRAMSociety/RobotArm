## Useful RASA commands to remember 
1) ``` rasa train nlu --nlu data/train.yml```(To train the nlu model)
2) ```rasa shell``` 
3) ```rasa test nlu --nlu tests/test.yml``` ([Testing Your Assistant (rasa.com)](https://rasa.com/docs/rasa/testing-your-assistant/))
	This outputs the data in a results folder. Important to metric to look at is the confusion matrix to see where its messing up. 

## Possible Config Files
1) Useful Resources: https://rasa.com/docs/rasa/components