import React, { Component } from 'react';
import { connect } from 'react-redux';
import { Button, Modal, ModalHeader, ModalBody, ModalFooter, Container, Row, Col, Input } from 'reactstrap';

import droneActions from "redux/actions/droneActions";

const setpointMsgs = {
  gimbal: 'GIMBAL_SETPOINT',
  deployment: 'DEPLOYMENT_MOTOR_SETPOINT',
  latch: 'LATCH_SETPOINT',
  hotwire: 'HOTWIRE_SETPOINT'
}
const mapStateToProps = state => { 
  return {
    missionCommands: state.mission.commands,
    missionCompiled: state.mission.missionCompiled,
    missionUploaded: state.mission.missionUploaded,
    dropReady: state.telemetry.droneTelemetry ? state.telemetry.droneTelemetry.output.deploy : null,
    lastDroppyCommand: state.mission.lastDroppyCommand,
    runningMission: state.telemetry.droneTelemetry ? (state.telemetry.droneTelemetry.output.state === 5) : null, // 5 is MISSION in flight loop state machine
    setpoints: state.telemetry.setpoints
  }; 
};

// Makes an object with the same function names as droneActions, 
// but with every action creator wrapped into a dispatch call.
const mapDispatchToProps = droneActions;

class DroneActions extends Component {
  state = {
    modal: false,
    message: "",
    setpointModal: false,
    setpointInputs: {
      gimbal: 0.0,
      deployment: 0.0,
      latch: false,
      hotwire: false
    }
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

  toggleSetpoints = () => {
    this.setState({setpointModal: !this.state.setpointModal});
  }

  booleanSetpointChanged = (e) => {
    this.setState({ setpointInputs: {...this.state.setpointInputs, [e.target.name]: e.target.checked} });
  }

  floatSetpointChanged = (e) => {
    let newValue = Number(e.target.value);
    if (!isNaN(newValue)) {
      this.setState({ setpointInputs: {...this.state.setpointInputs, [e.target.name]: newValue} });
    }
  }

  sendSetpoint = (e) => {
    this.props.sendSetpoint(setpointMsgs[e.target.name], this.state.setpointInputs[e.target.name]);
  }

  render() {
    const { message } = this.state;
    return (
      <span className="DroneActions">        
        <div className="buttonArray">
          {this.props.missionCompiled ?
            <button id="runMissionButton" onClick={()=>this.toggleWithName("Upload Mission", this.props.uploadMission)} /*disabled={this.props.missionUploaded}*/>Upload Mission</button>
          :
            <button id="runMissionButton" onClick={()=>this.toggleWithName("Compile Mission", this.compileMission)} disabled={this.props.runningMission}>Compile Mission</button>
          }
          {/* {this.props.missionStatus === 'RUN_MISSION' ? 
            <button id="takeoffButton" onClick={()=>this.toggleWithName("Pause Mission", this.props.pauseMission)}>Pause Mission</button>
          : this.props.missionUploaded || this.props.missionStatus === 'PAUSE_MISSION' ?
            <button id="runMissionButton" onClick={()=>this.toggleWithName("Run Mission", this.props.runMission)}>Run Mission</button> 
          :
            <button id="runMissionButton" disabled>Run Mission</button>
          } */}
          {/* <button id="failsafeButton" onClick={()=>this.toggleWithName("End Mission", this.props.endMission)}>End Mission</button> */}
          <button id="landButton" disabled={!this.props.dropReady} onClick={()=>this.toggleWithName("Start UGV Drop", this.props.droppyStart)}>Start Drop</button>
          <button id="takeoffButton" disabled={this.props.lastDroppyCommand != null} onClick={()=>this.toggleWithName("Cut Wire & Drive", this.props.droppyCut)}>{"Cut Wire & Drive"}</button>

          {/* <button id="takeoffButton" onClick={()=>this.toggleWithName("Takeoff", this.props.droneTakeoff)}>Takeoff</button> */}
          {/* <button id="landButton" onClick={()=>this.toggleWithName("Land", this.props.droneLand)}>Land</button> */}
          {/* <button id="failsafeButton" onClick={()=>this.toggleWithName("Failsafe Landing", this.props.droneFailsafe)}>Failsafe Landing</button> */}
          {/* <button id="throttleCutButton" onClick={()=>this.toggleWithName("Throttle Cut", this.props.droneThrottleCut)}>Throttle Cut</button> */}
          <button id="failsafeButton" onClick={()=>this.toggleWithName("Drive UGV", this.props.driveUgv)}>Drive UGV</button>
          <button id="throttleCutButton" onClick={()=>this.toggleWithName("Disable UGV", this.props.disableUgv)}>Disable UGV</button>
          <button id="landButton" onClick={this.toggleSetpoints}>Setpoints</button>
          <Modal isOpen={this.state.modal} toggle={this.toggle} className="DroneActions">
          <ModalHeader toggle={this.toggle}>WARNING</ModalHeader>
          <ModalBody>
            Are you sure you want to {message}?
          </ModalBody>
          <ModalFooter>
            <Button color="primary" onClick={this.doAction}>{message}</Button>
            <Button color="secondary" onClick={this.toggle}>Cancel</Button>
          </ModalFooter>
        </Modal>

        <Modal isOpen={this.state.setpointModal} toggle={this.toggleSetpoints} className="DroneActions">
          <ModalHeader toggle={this.toggleSetpoints}>Setpoint Override</ModalHeader>
          <ModalBody>
            <Container>
              <Row>
                <Col xs="6">Gimbal: <b>{this.props.setpoints.gimbal}</b></Col>
                <Col><Input name='gimbal' type="number" value={this.state.setpointInputs.gimbal} onChange={this.floatSetpointChanged}/></Col>
                <Col><Button name='gimbal' color="primary" onClick={this.sendSetpoint}>Send!</Button></Col>
              </Row>
              <Row>
                <Col xs="6">Deployment Motor: <b>{this.props.setpoints.deployment}</b></Col>
                <Col><Input name='deployment' type="number" value={this.state.setpointInputs.deployment} onChange={this.floatSetpointChanged}/></Col>
                <Col><Button name='deployment' color="primary" onClick={this.sendSetpoint}>Send!</Button></Col>
              </Row>
              <Row>
                <Col xs="6">Deployment Latch: <b>{this.props.setpoints.latch != null ? (this.props.setpoints.latch ? "True": "False") : null}</b></Col>
                <Col><Input name='latch' type="checkbox" checked={this.state.setpointInputs.latch} onChange={this.booleanSetpointChanged}/></Col>
                <Col><Button name='latch' color="primary" onClick={this.sendSetpoint}>Send!</Button></Col>
              </Row>
              <Row>
                <Col xs="6">Hotwire: <b>{this.props.setpoints.hotwire != null ? (this.props.setpoints.hotwire ? "True": "False") : null}</b></Col>
                <Col><Input name='hotwire' type="checkbox" checked={this.state.setpointInputs.hotwire} onChange={this.booleanSetpointChanged}/></Col>
                <Col><Button name='hotwire' color="primary" onClick={this.sendSetpoint}>Send!</Button></Col>
              </Row>
            </Container>
          </ModalBody>
          <ModalFooter>
            <Button color="secondary" onClick={this.toggleSetpoints}>Close</Button>
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
