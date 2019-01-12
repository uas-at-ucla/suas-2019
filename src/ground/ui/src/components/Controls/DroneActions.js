import React, { Component } from 'react';
import { connect } from 'react-redux';

import droneActions from "../../actions/droneActions";

const mapStateToProps = state => { return {}; };

// Makes an object with the same function names as droneActions, 
// but with every action creator wrapped into a dispatch call.
const mapDispatchToProps = droneActions;

class DroneActions extends Component {
  render() {
    return (
      <div className="DroneActions">
        <span>Change Drone State</span>
        <button onClick={this.props.droneTakeoff}>TAKEOFF</button>
        <button onClick={this.props.droneLand}>LAND</button>
        <button onClick={this.props.droneFailsafe}>FAILSAFE</button>
        <button onClick={this.props.droneThrottleCut}>THROTTLE_CUT</button>
      </div>
    );
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(DroneActions);
