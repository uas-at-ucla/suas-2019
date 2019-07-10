import React from "react";

import "./UasLogo.css";
import Logo from "graphics/uasLogo.svg";

class UasLogo extends React.Component {
  public render() {
    return <img className="logo-img" src={Logo} alt="" />;
  }
}

export default UasLogo;
