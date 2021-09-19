import React, { useState } from 'react';
import "./recordbtn.css"

interface RecordBtnProps{
    active: boolean
}
const RecordBtn =({active}:RecordBtnProps) => {
    if (!active) {
        return (
            <div className="recbtncircle">
                <div>
                    <img className="recbtnicon" src={"https://cdn-icons-png.flaticon.com/512/2282/2282210.png"} alt="RecBtn" />
                </div>
            </div>
        )
    }
    else{
        return (
            <>
            <h1> Listening ...</h1>
            <div className="recbtnactive">
                <div>
                    <img className="recbtnicon" src={"https://cdn-icons-png.flaticon.com/512/2282/2282210.png"} alt="RecBtn" />
                </div>
            </div>
          
            </>
        )
    }

}

export default RecordBtn