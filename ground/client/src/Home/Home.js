import React, { Component } from 'react';
import './Home.css';
import Sidebar from '../Sidebar/Sidebar'
import Map from '../Map/Map'
import Telemetry from '../Telemetry/Telemetry'
import DarkModal from '../DarkModal/DarkModal'
import logo from '../graphics/vector_logo.svg';

class Home extends Component {
	state = {
		isSidebarShown: true,
    followDrone: true,
    mission: this.props.appState.missions[0] || null,
    waypoints: []
	}

  componentWillReceiveProps(nextProps) {
    if (!this.state.mission && nextProps.appState.missions.length > 0) {
      this.setState({mission: nextProps.appState.missions[0]});
    }
  }

  render() {
    return (
      <div className="Home">
        <Map id="map" appState={this.props.appState}
             homeState={this.state} setHomeState={this.setHomeState}/>
        <div id="left_side">
          <div id="top_left">
            <img id="logo" src={logo} width="270px" alt=""/>
            <div id="map_buttons">
              <div>
                <button id="follow_drone_btn" className="btn btn-dark" onClick={this.followDrone}>
                  <i className="fa fa-location-arrow" aria-hidden="true"></i>
                </button>
              </div>
              <div>
                <button id="sidebar_btn" className="btn btn-dark" onClick={this.toggleSidebar}>
                  { !this.state.isSidebarShown ? <i className="fa fa-bars" aria-hidden="true"></i> : '✕' }
                </button>
              </div>
            </div>
          </div>
          <div id="sidebar_container" className={!this.state.isSidebarShown ? 'hidden' : null}>
            <Sidebar appState={this.props.appState}
                       homeState={this.state} setHomeState={this.setHomeState}
                       socketEmit={this.props.socketEmit}/>
          </div>
        </div>
        <div id="right_side">
          <Telemetry appState={this.props.appState}/>
          <button id="failsafe_prompt_btn" className="btn btn-danger" data-toggle="modal" data-target="#failsafe_modal">
            Failsafe
          </button>
          <DarkModal myId="failsafe_modal" myTitle="Confirm Failsafe">
            <button id="failsafe_btn" className="btn btn-danger btn-lg" data-dismiss="modal" onClick={this.failsafe}>
              Failsafe
            </button>
          </DarkModal>
        </div>
      </div>
    );
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

  failsafe = () => {
    console.log('Failsafe button clicked!');
    this.props.socketEmit('failsafe');
  }
}

export default Home;