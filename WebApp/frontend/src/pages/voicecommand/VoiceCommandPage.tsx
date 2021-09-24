import React, { useState, useEffect } from "react";
import ConfirmMessage from "./components/confirmmsg/confirmmsg";
import RecordBtn from "./components/recordbtn/recordbtn";
import "./VoiceCommandPage.css";
interface dataObject {
  message: string;
}

function VoiceCommandPage() {
  const [isRecording, setIsRecording] = useState(false);
  const [data, setData] = useState<dataObject>();
  const [typedMessage, setTypedMessage] = useState("");
  const [isTyped, setIsTyped] = useState(false); 
  const recordHandler = () => {
    setIsRecording(true);
    fetch("http://127.0.0.1:5000/record")
      .then((response) => response.json())
      .then(function (myJson) {
        setData(myJson);
        setIsRecording(false);
      });
  };

  const submitHandler = () => {
    console.log(typedMessage)
    setIsTyped(true)
  
  };

  if (data) {
    return (
      <div>
        <div onClick={recordHandler}>
          <RecordBtn active={isRecording} />
        </div>
        <ConfirmMessage message={data.message} />
      </div>
    );
  }

  if (isTyped) {
    return (
      <div>
        <div onClick={recordHandler}>
          <RecordBtn active={isRecording} />
        </div>
        <ConfirmMessage message={typedMessage} />
      </div>
    );
  }

  return (
    <div>
      <div className="recordbutton" onClick={recordHandler}>
        <RecordBtn active={isRecording} />
      </div>
      <p> OR </p>
      <h1> Enter Command</h1>
      <input
        type="text"
        onChange={(text) => setTypedMessage(text.target.value)}
        defaultValue={"Enter command"}
      />
      <button type="button" onClick={submitHandler}>
        {" "}
        Submit{" "}
      </button>
    </div>
  );
}
export default VoiceCommandPage;
