import React from 'react';
import CentralCircle from './components/centralcircle/centralcircle';
import DashCircle from './components/dashcircle/dashcircle';
import IconCard from './components/iconcard/iconcard';
import "./homepage.css"
function Homepage() {
    return (
        <div className="hmback">
            <CentralCircle/>
            <DashCircle />
            <div className="ic1">
                <IconCard img={"https://cdn-icons-png.flaticon.com/512/1005/1005531.png"}/>
            </div>
            <div className="ic2">
                <IconCard  img={"https://cdn-icons-png.flaticon.com/512/4829/4829729.png"} />
            </div>
            <div className="ic3">
                <IconCard  img={"https://cdn-icons-png.flaticon.com/512/3079/3079162.png"}/>
            </div>
            <div className="ic4">
                <IconCard  img={"https://cdn-icons-png.flaticon.com/512/680/680033.png"}/>
            </div>

        </div>
    );
}
export default Homepage