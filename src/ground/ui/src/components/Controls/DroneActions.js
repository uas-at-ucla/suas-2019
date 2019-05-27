import React, { Component } from 'react';
import { connect } from 'react-redux';
import { Button, Modal, ModalHeader, ModalBody, ModalFooter } from 'reactstrap';

import droneActions from "redux/actions/droneActions";

const mapStateToProps = state => { 
  return {
    missionCommands: state.mission.commands,
    missionUploaded: state.mission.missionUploaded
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

  toggleWithName = (message) => {
    this.setState({ message: message, modal: !this.state.modal });
  }

  doAction = () => {
    switch (this.state.message)
    { 
      case "Compile Mission" : this.compileMission(); 
      break;
      case "Run Mission" : this.props.runMission();
      break;
      case "Takeoff" : this.props.droneTakeoff();
      break;
      case "Land" : this.props.droneLand();
      break;
      case "Failsafe Landing": this.props.droneFailsafe();
      break;
      case "Throttle Cut":  this.props.droneThrottleCut();
      break; 
      case "Drive UGV":  this.props.driveUgv();
      break; 
    }
    this.toggle();
  }

  render() {
    const { message } = this.state;
    return (
      <span className="DroneActions">        
        <div className="buttonArray">
          {this.props.missionUploaded ? 
            <button id="runMissionButton" onClick={()=>this.toggleWithName("Run Mission")}>Run Mission</button>
          :
            <button id="runMissionButton" onClick={()=>this.toggleWithName("Compile Mission")}>Compile Mission</button>
          }
          <button id="takeoffButton" onClick={()=>this.toggleWithName("Takeoff")}>Takeoff</button>
          <button id="landButton" onClick={()=>this.toggleWithName("Land")}>Land</button>
          <button id="failsafeButton" onClick={()=>this.toggleWithName("Failsafe Landing")}>Failsafe Landing</button>
          <button id="throttleCutButton" onClick={()=>this.toggleWithName("Throttle Cut")}>Throttle Cut</button>
          <button id="takeoffButton" onClick={()=>this.toggleWithName("Drive UGV")}>Drive UGV</button>
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
