import React from 'react';
import DashCircle from './components/iconcard/dashcircle/dashcircle';
import IconCard from './components/iconcard/iconcard';
import "./homepage.css"
function Homepage() {
    return (
        <div className="background">

            <DashCircle />
            <div className="ic1">
                <IconCard />
            </div>
            <div className="ic2">
                <IconCard />
            </div>
            <div className="ic3">
                <IconCard />
            </div>
            <div className="ic4">
                <IconCard />
            </div>

        </div>
    );
}
export default Homepage