import React, { Component } from 'react';
import { connect } from 'react-redux';
import { Button, Modal, ModalHeader, ModalBody, ModalFooter } from 'reactstrap';

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
  constructor(props) {
    super(props);
  this.state = {
    modal: false,
    message: ""
  };
  this.toggle = this.toggle.bind(this);
  this.toggleWithName = this.toggleWithName.bind(this);
  }
  
  toggle() {
    this.setState(prevState => ({
      modal: !prevState.modal
    }));
  }

  toggleWithName(message){
    this.setState({message: message });
    this.toggle();
  }

  doAction(action){
    switch (action)
    { 
     case "Run Mission" : this.runMission();
     break;
     case "Takeoff" : this.props.droneTakeoff();
     break;
     case "Land" : this.props.droneLand();
     break;
     case "Failsafe Landing": this.props.droneFailsafe();
     break;
     case "Throttle Cut":  this.props.droneThrottleCut();
     break; 
     default: this.props.droneLand();
     break; 
    }
  }

  render() {
    const { message } = this.state;
    return (
      <span className="DroneActions">        
        <div className="buttonArray">
          <button id="runMissionButton" onClick={()=>this.toggleWithName("Run Mission")}>Run Mission</button>
          <button id="takeoffButton" onClick={()=>this.toggleWithName("Takeoff")}>Takeoff</button>
          <button id="landButton" onClick={()=>this.toggleWithName("Land")}>Land</button>
          <button id="failsafeButton" onClick={()=>this.toggleWithName("Failsafe Landing")}>Failsafe Landing</button>
          <button id="throttleCutButton" onClick={()=>this.toggleWithName("Throttle Cut")}>Throttle Cut</button>
          <Modal isOpen={this.state.modal} toggle={this.toggle} className={this.props.className}>
          <ModalHeader toggle={this.toggle}>WARNING</ModalHeader>
          <ModalBody>
            Are you sure you want to {message}?
          </ModalBody>
          <ModalFooter>
            <Button color="primary" onClick={this.toggle}>{message}</Button>{' '}
            <Button color="secondary" onClick={this.toggle}>Cancel</Button>
          </ModalFooter>
        </Modal>
        </div>
      </span>
    );
  }


  runMission = () => {
    this.props.runMission(this.props.missionCommands);
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(DroneActions);
