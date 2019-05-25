import React, { Component } from 'react';
import { connect } from 'react-redux';

import droneActions from "redux/actions/droneActions";

const mapStateToProps = state => { 
  return {
    missionCommands: state.mission.commands
  }; 
};

// Makes an object with the same function names as droneActions, 
// but with every action creator wrapped into a dispatch call.
const mapDispatchToProps = droneActions;

class DroneActions extends Component {
  render() {
    return (
      <span className="DroneActions">        
        <div className="buttonArray">
          <button id="runMissionButton" onClick={this.runMission}>Run Mission</button>
          <button id="takeoffButton" onClick={this.props.droneTakeoff}>Takeoff</button>
          <button id="landButton" onClick={this.props.droneLand}>Land</button>
          <button id="failsafeButton" onClick={this.props.droneFailsafe}>Failsafe Landing</button>
          <button id="throttleCutButton" onClick={this.props.droneThrottleCut}>Throttle Cut</button>
          <button id="takeoffButton" onClick={this.props.driveUgv}>Drive UGV</button>
        </div>
      </span>
    );
  }

  runMission = () => {
    this.props.runMission(this.props.missionCommands);
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(DroneActions);
