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
      <tr>
        <td>{waypoint.type}</td>
        <td>{waypoint.lat.toFixed(5)}</td>
        <td>{waypoint.lng.toFixed(5)}</td>
        <td>{waypoint.alt.toFixed(1)} m</td>
      </tr>
    );

    return (
      <div>
        <table id="waypointListLabelsTable">
          <tr id="waypointListLabels">
            <td>Type</td>
            <td>Latitude</td>
            <td>Longitude</td>
            <td>Altitude</td>
          </tr>
        </table>
        <div class="scrollbar">
          <table id="waypointList">
            {waypointsList}
          </table>
        </div>

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
      type: "goto"
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
