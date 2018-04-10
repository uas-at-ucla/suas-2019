import React, { Component } from "react";
import "./Sidebar.css";
import Panel from "./Panel";

class Sidebar extends Component {
  render() {
    return (
      <div className="Sidebar">
        <div id="sidebar_content">
          <Panel
            id="missionPlanner"
            title="Mission Plan"
            homeState={this.props.homeState}
            setHomeState={this.props.setHomeState}
            makeCommand={this.props.makeCommand}
            commandTypes={this.props.commandTypes}
          />
        </div>
      </div>
    );
  }
}

export default Sidebar;
