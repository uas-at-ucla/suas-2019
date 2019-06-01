import React, { Component } from 'react';
import { connect } from 'react-redux';
import { Button, Modal, ModalHeader, ModalBody, ModalFooter } from 'reactstrap';

import droneActions from "redux/actions/droneActions";

const mapStateToProps = state => { 
  return {
    missionCommands: state.mission.commands,
    missionCompiled: state.mission.missionCompiled,
    missionUploaded: state.mission.missionUploaded,
    missionStatus: state.mission.missionStatus,
  }; 
};

// Makes an object with the same function names as droneActions, 
// but with every action creator wrapped into a dispatch call.
const mapDispatchToProps = droneActions;

class DroneActions extends Component {
  state = {
    modal: false,
    message: ""
  };
  
  toggle = () => {
    this.setState(prevState => ({
      modal: !prevState.modal
    }));
  }

  toggleWithName = (message, action) => {
    this.setState({ 
      message: message,
      action: action,
      modal: !this.state.modal
    });
  }

  doAction = () => {
    this.state.action();
    this.toggle();
  }

  render() {
    const { message } = this.state;
    return (
      <span className="DroneActions">        
        <div className="buttonArray">
          {this.props.missionStatus === 'PAUSE_MISSION' ? 
            <button id="runMissionButton" onClick={()=>this.toggleWithName("Run Mission", this.props.runMission)}>Run Mission</button>
          : this.props.missionStatus === 'RUN_MISSION' ? 
            <button id="takeoffButton" onClick={()=>this.toggleWithName("Pause Mission", this.props.pauseMission)}>Pause Mission</button>
          : this.props.missionCompiled && this.props.missionUploaded ?
            <button id="runMissionButton" onClick={()=>this.toggleWithName("Run Mission", this.props.runMission)}>Run Mission</button>
          : this.props.missionCompiled ?
            <button id="runMissionButton" onClick={()=>this.toggleWithName("Upload Mission", this.props.uploadMission)}>Upload Mission</button>
          :
            <button id="runMissionButton" onClick={()=>this.toggleWithName("Compile Mission", this.compileMission)}>Compile Mission</button> 
          }
          <button id="failsafeButton" onClick={()=>this.toggleWithName("End Mission", this.props.endMission)}>End Mission</button>
          <button id="takeoffButton" onClick={()=>this.toggleWithName("Takeoff", this.props.droneTakeoff)}>Takeoff</button>
          <button id="landButton" onClick={()=>this.toggleWithName("Land", this.props.droneLand)}>Land</button>
          <button id="failsafeButton" onClick={()=>this.toggleWithName("Failsafe Landing", this.props.droneFailsafe)}>Failsafe Landing</button>
          <button id="throttleCutButton" onClick={()=>this.toggleWithName("Throttle Cut", this.props.droneThrottleCut)}>Throttle Cut</button>
          <button id="takeoffButton" onClick={()=>this.toggleWithName("Drive UGV", this.props.driveUgv)}>Drive UGV</button>
          <Modal isOpen={this.state.modal} toggle={this.toggle} className={this.props.className}>
          <ModalHeader toggle={this.toggle}>WARNING</ModalHeader>
          <ModalBody>
            Are you sure you want to {message}?
          </ModalBody>
          <ModalFooter>
            <Button color="primary" onClick={this.doAction}>{message}</Button>
            <Button color="secondary" onClick={this.toggle}>Cancel</Button>
          </ModalFooter>
        </Modal>
        </div>
      </span>
    );
  }

  compileMission = () => {
    this.props.compileMission(this.props.missionCommands);
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(DroneActions);
