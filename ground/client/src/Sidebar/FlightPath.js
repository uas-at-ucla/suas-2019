import React, { Component } from 'react';

class FlightPath extends Component {
  constructor(props) {
    super(props);

    this.state = {
      commandAddMove: false
    }
  }

  render() {
    const waypointsList = this.props.homeState.waypoints.map((waypoint) =>
      <li>{waypoint.lat + ', ' + waypoint.lng + ', ' + waypoint.alt + ' m'}</li>
    );

    return (
      <div>
        <ol id="waypoints_list">
          {waypointsList}
        </ol>

        <button className="btn btn-outline-primary"
                onClick={this.addMissionWaypoints}>
          Add Waypoint
        </button>

        <button className="btn btn-outline-success right"
                onClick={this.sendGotoWaypointsCommand}>
          Execute
        </button>
      </div>
    );
  }

  addMissionWaypoint(lat, lng) {
    this.setState({commandAddMove: true});

    let waypoints = this.props.homeState.waypoints.slice();

    let waypoint = {
      lat: lat,
      lng: lng,
      alt: 20,
      fromMission: true
    }

    waypoints.push(waypoint);

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

export default FlightPath
