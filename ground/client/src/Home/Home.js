import React, { Component } from 'react';
import './Home.css';
import Dashboard from '../Dashboard/Dashboard'
import Map from '../Map/Map'
import Telemetry from '../Telemetry/Telemetry'
import logo from '../images/vector_logo.svg';

class Home extends Component {
	state = {
		isSidebarShown: true,
    followDrone: true,
    mission: null,
    waypoints: []
	}

  componentWillReceiveProps(nextProps) {
    if (!this.state.mission && nextProps.app.state.missions.length > 0) {
      this.setState({mission: nextProps.app.state.missions[0]});
    }
  }

  render() {
    return (
      <div className="Home">
        <Map id="map" appState={this.props.app.state} 
          setHomeState={this.setHomeState} homeState={this.state}/>
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
          <div id="sidebar" ref="sidebar" className={!this.state.isSidebarShown ? 'hidden' : null}>
            <Dashboard app={this.props.app} home={this}/>
          </div>
        </div>
        <div id="right_side">
          <Telemetry app={this.props.app}/>
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
}

export default Home;