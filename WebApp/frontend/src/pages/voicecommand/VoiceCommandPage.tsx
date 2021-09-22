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

  const recordHandler = () => {
    setIsRecording(true);
    fetch("http://127.0.0.1:5000/record")
      .then((response) => response.json())
      .then(function (myJson) {
        setData(myJson);
        setIsRecording(false);
      });
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

  return (
    <div>
      <div onClick={recordHandler}>
        <RecordBtn active={isRecording} />
      </div>
    </div>
  );
}
export default VoiceCommandPage;
