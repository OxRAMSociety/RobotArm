import React, { useState } from "react";
import Prediction from "../prediction/prediction";

interface ConfirmMessageProps {
  message: string;
}

const ConfirmMessage = ({ message }: ConfirmMessageProps) => {
  const [confirm, setConfirm] = useState(false);
  const toggleConfirm = () => {
    setConfirm(true);
  };

  return (
    <div>
      <div>
        <p>Did I catch you correctly?</p>
        <p>{message}</p>
        <Prediction message={message} />
        <button onClick={toggleConfirm}>Confirm</button>
      </div>
    </div>
  );
};

export default ConfirmMessage;
