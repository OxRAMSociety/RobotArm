import React, { useState, useEffect } from 'react';




interface predictionObject{
  prediction: Array<any>
}
interface PredictionProps{
    message:string
}
const Prediction =({message}:PredictionProps) => {
  const [predictArr, setPredictArr] = useState<predictionObject>(); 

  useEffect(() => {
    fetch("http://127.0.0.1:5000/predict?message="+message)
    .then((response) => response.json())
    .then(function (myJson) {
      setPredictArr(myJson);
    })
  },[message])

  if(predictArr){
    return (
      <p>{predictArr.prediction[0]}</p>
    );
  }
  else{
    return(<div></div>)
  }
}

export default Prediction