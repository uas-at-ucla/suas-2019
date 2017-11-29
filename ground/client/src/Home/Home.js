import React, { Component } from 'react';
import './Home.css';
import Dashboard from '../Dashboard/Dashboard'
import Map from '../Map/Map'
import logo from '../images/vector_logo.svg';

class Home extends Component {
	state = {
		isSidebarShown: true,
    waypoints: []
	}

  render() {
    return (
      <div className="Home">
        <Map ref="map" id="map" setWaypoints={this.setWaypoints.bind(this)}/>
        <div id="left_side">
          <div id="top_left">
            <img id="logo" src={logo} width="270px"/>
            <div id="map_buttons">
              <div>
                <button id="follow_drone_btn" className="btn btn-dark" onClick={() => this.refs.map.followDrone()}>
                  <i className="fa fa-location-arrow" aria-hidden="true"></i>
                </button>
              </div>
              <div>
                <button id="sidebar_btn" className="btn btn-dark" onClick={this.toggleSidebar.bind(this)}>
                  { !this.state.isSidebarShown ? <i className="fa fa-bars" aria-hidden="true"></i> : '✕' }
                </button>
              </div>
            </div>
          </div>
          <div id="sidebar" ref="sidebar" className={!this.state.isSidebarShown ? 'hidden' : null}>
            <Dashboard map={this.refs.map} waypoints={this.state.waypoints}
            sendGotoWaypointsCommand={this.sendGotoWaypointsCommand.bind(this)}/>
          </div>
        </div>
        <div id="right_side">
          <div id="telemetry" className="card text-white">
            <div id="full_state">
              <span id="armed_indicator">{this.props.droneArmedStatus}</span>
              <span id="state_indicator">{this.props.droneState}</span>
            </div>
            <table>
              <tbody>
                <tr id="telemetry_important">
                  <td>Speed</td>
                  <td id="telemetry_speed">{this.props.telemetryText.speed}</td>
                </tr>
                <tr>
                  <td>Position</td>
                  <td id="telemetry_position">{this.props.telemetryText.position}</td>
                </tr>
                <tr>
                  <td>Heading</td>
                  <td id="telemetry_heading">{this.props.telemetryText.heading}</td>
                </tr>
                <tr>
                  <td>Altitude</td>
                  <td id="telemetry_altitude">{this.props.telemetryText.altitude}</td>
                </tr>
                <tr>
                  <td>Satellite Count</td>
                  <td id="telemetry_satellites">{this.props.telemetryText.satellites}</td>
                </tr>
             </tbody>
            </table>
          </div>
        </div>
      </div>
    );
  }

  setWaypoints(waypoints) {
    this.setState({waypoints: waypoints});
  }

  sendGotoWaypointsCommand() {
    var commands = [];
    for (var waypoint of this.state.waypoints) {
      commands.push({
        type: 'goto',
        pos: waypoint
      });
    }
    this.props.socket.emit('execute_commands', commands);
  }

  toggleSidebar() {
  	this.setState({isSidebarShown: !this.state.isSidebarShown});
  }
}

export default Home;