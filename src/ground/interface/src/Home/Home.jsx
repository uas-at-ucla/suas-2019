import React, { Component } from "react";

import "./Home.css";
import Sidebar from "../Sidebar/Sidebar";
import Map from "../Map/Map";
import Telemetry from "../Telemetry/Telemetry";
import Controls from "../Controls/Controls";

class Home extends Component {
  state = {
    isSidebarShown: true,
    mission: this.props.appState.missions[0] || null,
    commands: [],
    dontRedrawCommands: false,
    focusedCommand: null
  };

  render() {
    return (
      <div className="Home">
        <div id="sides">
          <Map
            id="map"
            appState={this.props.appState}
            setAppState={this.props.setAppState}
            homeState={this.state}
            setHomeState={this.setHomeState}
          />
          <div id="left_side">
            <div
              id="sidebar_container"
              className={!this.state.isSidebarShown ? "hidden" : null}
            >
              <Sidebar
                appState={this.props.appState}
                homeState={this.state}
                setHomeState={this.setHomeState}
                socketEmit={this.props.socketEmit}
              />
            </div>
          </div>
          <div id="right_side">
            <Telemetry appState={this.props.appState} homeState={this.state} />
          </div>
        </div>

        <Controls homeState={this.state} socketEmit={this.props.socketEmit} />
      </div>
    );
  }

  componentWillReceiveProps(nextProps) {
    if (nextProps.appState.missions.length > 0 && nextProps.appState.missions !== this.props.appState.missions) {
      this.setState({ mission: nextProps.appState.missions[0] });
    }
  }

  setHomeState = newState => {
    this.setState(newState);
  };

  toggleSidebar = () => {
    this.setState({ isSidebarShown: !this.state.isSidebarShown });
  };
}

export default Home;
