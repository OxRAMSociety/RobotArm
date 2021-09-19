import React, { useState, useEffect } from "react";
import RecordBtn from "./components/recordbtn/recordbtn";

import "./VoiceCommandPage.css";
interface dataObject{
  message: string
  prediction: Array<any>
}

function VoiceCommandPage() {
  const [isRecording, setIsRecording] = useState(false);
  const [data, setData] = useState<dataObject>();

 
  const getNLPData = () => {
    setIsRecording(true)
    fetch("http://127.0.0.1:5000/record")
      .then(response => response.json())
      .then(function (myJson) {
        setData(myJson);
        setIsRecording(false)
      });
  };

  const recordHandler = () => {  
    getNLPData();
  };

  if(data){
    console.log(data)
    return <div>
      <p>{data.message}</p>
      <p>{data.prediction}</p>
      </div>
  }

  return (
    <div onClick={recordHandler}>
      <RecordBtn active={isRecording} />
    </div>
  );
}
export default VoiceCommandPage;
