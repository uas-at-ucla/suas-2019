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

        <div className="buttonArray">
          <button id="takeoffButton" onClick={this.props.droneTakeoff}>Takeoff</button>
          <button id="landButton" onClick={this.props.droneLand}>Land</button>
          <button id="failsafeButton" onClick={this.props.droneFailsafe}>Failsafe Landing</button>
          <button id="throttleCutButton" onClick={this.props.droneThrottleCut}>Throttle Cut</button>
          <button id="runMission" onClick={this.props.droneRunMission}>Run Mission</button>
          <button id="Hold" onClick={this.props.droneHold}>Hold</button>
          <button id="Offboard" onClick={this.props.droneOffboard}>Offboard</button>
          <button id="RTL" onClick={this.props.droneRTL}>RTL</button>
          <button id="Land" onClick={this.props.droneLand}>Land</button>
          <button id="Arm" onClick={this.props.droneArm}>Arm</button>
          <button id="Disarm" onClick={this.props.droneDisarm}>Disarm</button>
          <button id="Alarm" onClick={this.props.droneAlarm}>Alarm</button>
          <button id="BombDrop" onClick={this.props.droneBombDrop}>Bomb Drop</button>
          <button id="DSLR" onClick={this.props.droneDSLR}>DLSR</button>

        </div>
      </div>
    );
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(DroneActions);
