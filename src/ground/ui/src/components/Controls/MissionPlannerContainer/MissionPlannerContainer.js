import React, { Component } from 'react';
import { Button, Modal, ModalHeader, ModalBody } from 'reactstrap';

import './MissionPlannerContainer.css';
import MissionPlanner from './MissionPlanner';

class MissionPlannerContainer extends Component {
  state = {
    expand: false
  }

  render() {
    return (
      <div className="MissionPlannerContainer">
        
        <div className="missionPlannerHeader"> 
          <h1> Mission Planner </h1>
          <Button onClick={this.expand}>Expand</Button>
        </div>
        
        
        <div className="mission-planner-div">
          <div className="SmallMissionPlanner">
            <MissionPlanner/>
          </div>
        </div>
        <Modal isOpen={this.state.expand} toggle={this.close} className="MissionPlannerContainer">
          <ModalHeader toggle={this.close}>Mission Planner</ModalHeader>
          <ModalBody>
            <div className="BigMissionPlanner">
              <MissionPlanner/>
            </div>
          </ModalBody>
        </Modal>
      </div>
    );
  }

  expand = () => this.setState({expand: true});

  close = () => this.setState({expand: false});
}

export default MissionPlannerContainer;