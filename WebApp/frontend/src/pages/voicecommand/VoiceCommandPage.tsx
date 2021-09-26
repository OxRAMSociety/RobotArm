import React, { useState, useEffect, useRef } from "react";
import ConfirmMessage from "./components/confirmmsg/confirmmsg";
import RecordBtn from "./components/recordbtn/recordbtn";
import "./VoiceCommandPage.css";
interface dataObject {
  message: string;
}

interface InputChangeInterface {
  target: HTMLInputElement;
}

function VoiceCommandPage() {
  const [isRecording, setIsRecording] = useState(false);
  const [data, setData] = useState<dataObject>();
  const [typedMessage, setTypedMessage] = useState("");
  const [isTyped, setIsTyped] = useState(false);

  const typedMessageForm = useRef<HTMLInputElement>(null);

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
    if (typedMessageForm.current !== null) {
      setTypedMessage(typedMessageForm.current.value);
      setIsTyped(true);
    }
  };

  if (data) {
    return (
      <div className="vcpfullpage">
        <div className="recordbutton" onClick={recordHandler}>
          <RecordBtn active={isRecording} />
        </div>
        <p className="vcpp"> OR </p>
        <p className="vcpp"> Enter Command</p>

        <div className="vcpinput">
          <input
            type="text"
            ref={typedMessageForm}
            // onChange={(e)=>{setTypedMessage(e.target.value)}}
          />
          <button type="button" onClick={submitHandler} className="submitBtn">
            {" "}
            Submit{" "}
          </button>
        </div>
        <ConfirmMessage message={data.message} />
      </div>
    );
  }

  if (isTyped) {
    return (
      <div className="vcpfullpage">
        <div className="recordbutton" onClick={recordHandler} >
          <RecordBtn active={isRecording} />
        </div>
        <p className="vcpp"> OR </p>
        <p className="vcpp"> Enter Command</p>

        <div className="vcpinput">
          <input
            type="text"
            ref={typedMessageForm}
            // onChange={(e)=>{setTypedMessage(e.target.value)}}
          />
          <button type="button" onClick={submitHandler} className="submitBtn">
            {" "}
            Submit{" "}
          </button>
        </div>
        <ConfirmMessage message={typedMessage} />
      </div>
    );
  }

  return (
    <div className="vcpfullpage">
      <div className="recordbutton" onClick={recordHandler}>
        <RecordBtn active={isRecording} />
      </div>
      <p className="vcpp"> OR </p>
      <p className="vcpp"> Enter Command</p>

      <div className="vcpinput">
        <input
          type="text"
          ref={typedMessageForm}
          // onChange={(e)=>{setTypedMessage(e.target.value)}}
        />
        <button type="button" onClick={submitHandler} className="submitBtn">
          {" "}
          Submit{" "}
        </button>
      </div>
    </div>
  );
}
export default VoiceCommandPage;
