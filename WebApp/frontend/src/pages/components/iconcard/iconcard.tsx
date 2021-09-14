import React from 'react';
import "./iconcard.css"

interface IconCardProps{
    img: string
}
const IconCard =({img}: IconCardProps)=> {
    return (
        <div className="iccircle">
            <div>
                <img className="icicon" src={img} alt="Icon" />
            </div>
        </div>
    )

}

export default IconCard