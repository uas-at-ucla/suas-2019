import React, { Component } from 'react';
import { connect } from 'react-redux';
import droneActions from "../../actions/droneActions"

import "./Controls.css"

const mapStateToProps = state => { return {}; };

// Makes an object with the same function names as droneActions, 
// but with every action creator wrapped into a dispatch call.
const mapDispatchToProps = droneActions;

class DroneActions extends Component {
  render() {
    return (
      <div className="droneActions">
        <span>Change Drone State</span>
        
        <div className="buttonArray">
          <button id="takeoffButton" onClick={this.props.droneTakeoff}>Takeoff</button>
          <button id="landButton" onClick={this.props.droneLand}>Land</button>
          <button id="failsafeButton" onClick={this.props.droneFailsafe}>Failsafe Landing</button>
          <button id="throttleCutButton" onClick={this.props.droneThrottleCut}>Throttle Cut</button>
        </div>
      </div>
    );
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(DroneActions);
