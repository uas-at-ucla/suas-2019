import React, { Component } from 'react';
import './Sidebar.css'
import Panel from './Panel'
import MissionSelect from './MissionSelect'
import FlightPath from './FlightPath'

class Sidebar extends Component {

  render() {
    return (
      <div className="Sidebar">
        <div id="sidebar_content">
          <Panel>
            <MissionSelect appState={this.props.appState}
                           homeState={this.props.homeState}
                           setHomeState={this.props.setHomeState}/>
          </Panel>
          <Panel title="Flight Path">
            <FlightPath socketEmit={this.props.socketEmit}
                        homeState={this.props.homeState}
                        setHomeState={this.props.setHomeState}/>
          </Panel>
        </div>
        <div className="card text-white">
          <h4 className="card-header">Received Images</h4>
          <div className="card-body">
            <p className="card-text">Feature coming... eventually</p>
          </div>
        </div>
      </div>
    );
  }

  addMissionWaypoints = () => {
    let waypoints = this.props.homeState.waypoints.slice();
    let mission_waypoints = this.props.homeState.mission.mission_waypoints;
    for (let mission_waypoint of mission_waypoints) {
      let waypoint = {
        lat: mission_waypoint.latitude,
        lng: mission_waypoint.longitude,
        alt: mission_waypoint.altitude_msl,
        fromMission: true
      }
      waypoints.push(waypoint);
    }
    this.props.setHomeState({waypoints: waypoints});
  }

  sendGotoWaypointsCommand = () => {
    let commands = [];
    for (let waypoint of this.props.homeState.waypoints) {
      commands.push({
        type: 'goto',
        pos: waypoint
      });
    }
    this.props.socketEmit('execute_commands', commands);
  }
}

export default Sidebar
