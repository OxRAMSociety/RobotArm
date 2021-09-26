import React, { useState } from "react";
import Prediction from "../prediction/prediction";
import "./confirmmsg.css"

interface ConfirmMessageProps {
  message: string;
}

const ConfirmMessage = ({ message }: ConfirmMessageProps) => {
  const [confirm, setConfirm] = useState(false);
  const toggleConfirm = () => {
    setConfirm(true);
  };

  console.log("rerendered")
  return (
    <div className="confirmMessage">
      <div>
        <h4 className="confirmPrompt">Did I catch you correctly?</h4>
        <p>{message}</p>
        <Prediction message={message} />
        <button onClick={toggleConfirm} className="submitBtn">Confirm</button>
      </div>
    </div>
  );
};

export default ConfirmMessage;
