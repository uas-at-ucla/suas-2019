import React, { Component } from "react";
import Dashboard from "./Dashboard";
import Screen from "./Screen";

class Home extends Component {

  render() {
    return(
      <div className="Home">
        <div id="sidebar_container">
          <div id="sidebar">
            <Dashboard/>
          </div>
        </div>
        <Screen/>
      </div>
    );
  }
}

export default Home;