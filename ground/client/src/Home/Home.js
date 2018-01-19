import React, { Component } from 'react';
import './Home.css';
import Sidebar from '../Sidebar/Sidebar'
import Map from '../Map/Map'
import Telemetry from '../Telemetry/Telemetry'
import Controls from '../Controls/Controls'
import logo from '../graphics/vector_logo.svg';
import { Button } from 'react-bootstrap';
import testMission from './test_mission'

const USE_TEST_MISSION = true;

class Home extends Component {
  state = {
    isSidebarShown: true,
    followDrone: true,
    mission: USE_TEST_MISSION ? testMission : (this.props.appState.missions[0] || null),
    commands: [],
    dontRedrawCommands: false,
    focusedCommand: null
  }

  render() {
    return (
      <div className = "Home">
        <div id="sides">
          <Map
            id="map"
            appState={this.props.appState}
            homeState={this.state}
            setHomeState={this.setHomeState} />
          <div id="left_side">
            <div id="top_left">
              <img id="logo" src={logo} width="380px" onClick={this.followDrone}/>
              <div id="map_buttons">
                <Button id="follow_drone_btn"
                        bsStyle="primary"
                        onClick={this.followDrone}>
                  <i className="fa fa-location-arrow" aria-hidden="true"></i>
                </Button>
              </div>
            </div>
            <div id="sidebar_container"
                 className={!this.state.isSidebarShown ? 'hidden' : null}>
              <Sidebar
                appState={this.props.appState}
                homeState={this.state}
                setHomeState={this.setHomeState}
                socketEmit={this.props.socketEmit}/>
            </div>
          </div>
          <div id="right_side">
            <Telemetry appState={this.props.appState}
                       homeState={this.state}/>
          </div>
        </div>

        <Controls
          homeState={this.state}
          socketEmit={this.props.socketEmit}/>
      </div>
    );
  }

  componentWillReceiveProps(nextProps) {
    if (!this.state.mission && nextProps.appState.missions.length > 0) {
      this.setState({mission: nextProps.appState.missions[0]});
    }
  }

  setHomeState = (newState) => {
    this.setState(newState);
  }

  followDrone = () => {
    this.setState({followDrone: true});
  }

  toggleSidebar = () => {
    this.setState({isSidebarShown: !this.state.isSidebarShown});
  }
}

export default Home;
