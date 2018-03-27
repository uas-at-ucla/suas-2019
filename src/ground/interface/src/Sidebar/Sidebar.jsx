import React, { Component } from "react";
import "./Sidebar.css";
import Panel from "./Panel";
import MissionPlanner from "./MissionPlanner";

class Sidebar extends Component {
  render() {
    return (
      <div className="Sidebar">
        <div id="sidebar_content">
          <Panel id="missionPlanner">
            <MissionPlanner
              homeState={this.props.homeState}
              setHomeState={this.props.setHomeState}
            />
          </Panel>
        </div>
      </div>
    );
  }
}

export default Sidebar;
