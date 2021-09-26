import React from "react";
import { Link } from "react-router-dom";
import CentralCircle from "./components/centralcircle/centralcircle";
import DashCircle from "./components/dashcircle/dashcircle";
import IconCard from "./components/iconcard/iconcard";
import "./homepage.css";
function Homepage() {
  return (
    <div className="hmback">
      <CentralCircle />
      <DashCircle />
      <div className="labelpos1">
        <p className="label">Voice Command</p>
      </div>
      <div className="labelpos2">
        <p className="label">Movement</p>
      </div>
      <div className="labelpos3">
        <p className="label">Planning</p>
      </div>
      <div className="labelpos4">
        <p className="label">Vision</p>
      </div>
      <div className="ic1">
        <Link to="/voice-command">
          <IconCard
            img={"https://cdn-icons-png.flaticon.com/512/2861/2861905.png"}
          />
        </Link>
      </div>
      <div className="ic2">
        <IconCard
          img={"https://cdn-icons-png.flaticon.com/512/4829/4829729.png"}
        />
      </div>
      <div className="ic3">
        <IconCard
          img={"https://cdn-icons-png.flaticon.com/512/3079/3079162.png"}
        />
      </div>
      <div className="ic4">
        <IconCard
          img={"https://cdn-icons-png.flaticon.com/512/680/680033.png"}
        />
      </div>
    </div>
  );
}
export default Homepage;
