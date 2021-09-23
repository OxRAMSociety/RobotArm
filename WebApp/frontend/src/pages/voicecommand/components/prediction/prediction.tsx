import React, { useState, useEffect } from "react";


interface predictionObject {
  prediction: Array<any>;
}
interface PredictionProps {
  message: string;
}

const Prediction = ({ message }: PredictionProps) => {
  const [predictJson, setPredictJson] = useState<predictionObject>();
  const [var1, setVar1] = useState("");
  useEffect(() => {
    fetch("http://127.0.0.1:5000/predict?message=" + "move knight to A2")
      .then((response) => response.json())
      .then(function (myJson) {
        setPredictJson(myJson);
      });
  }, [message]);

  if (predictJson) {
    console.log(var1)
    return (
      <div>
        {predictJson.prediction.map((item) => (
          <div>
            <p>{item.title}</p>
            <input key={item.title} type="text" defaultValue={item.value}/>
          </div>
        ))}
      </div>
    );
  } else {
    return <div></div>;
  }
};

export default Prediction;
