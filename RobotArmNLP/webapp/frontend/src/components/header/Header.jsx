import React from "react";
import { MDBRow, MDBCol } from "mdbreact";
import Logo from "../../images/logo/RobotArmLogo.png";
import styles from "./Header.module.css";
const Header = (props) => {
  return (
    <div className={styles.header}>
      <MDBRow>
        <MDBCol><h1 className={styles.text}>{props.text}</h1></MDBCol>
        <MDBCol>
          <img src={Logo} className={styles.logo} />
        </MDBCol>
      </MDBRow>
    </div>
  );
};

export default Header;
