import React, { Component } from 'react';
import { connect } from 'react-redux';
import {test, change_drone_state, DroneStates} from "../../actions/exampleActions"

const mapStateToProps = state => {
  return {
    telemetry: state.telemetry
  };
};

const mapDispatchToProps = dispatch => {
  return {
    onTest: () =>{
      dispatch(test("hi"))
    },
    //idk if this is right bc it looks kinda extra...
    droneTakeoff: () => {
      dispatch(change_drone_state(DroneStates.TAKEOFF));
    },
    droneLand: () => {
      dispatch(change_drone_state(DroneStates.LAND));
    },
    droneFailsafe: () => {
      dispatch(change_drone_state(DroneStates.FAILSAFE));
    },
    droneThrottleCut: () => {
      dispatch(change_drone_state(DroneStates.THROTTLE_CUT));
    },

  };
};

class Telemetry extends Component {
  render() {
    return (
      <div>
        <span> Change Drone State </span>
         <button onClick = {this.props.droneTakeoff}>TAKEOFF</button>
         <button onClick = {this.props.droneLand}>LAND</button>
         <button onClick = {this.props.droneFailsafe}>FAILSAFE</button>
         <button onClick = {this.props.droneThrottleCut}>THROTTLE_CUT</button>

        <div className="Telemetry">

          {this.props.telemetry}
        </div>
      </div>
    );
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(Telemetry);
