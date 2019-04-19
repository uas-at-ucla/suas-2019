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
          <h1>Mission Planner</h1>
          <Button onClick={this.expand}>Expand</Button>
        </div>
        <div className="SmallMissionPlanner">
          <MissionPlanner className="SmallMissionPlanner"/>
        </div>
        
        <Modal isOpen={this.state.expand} toggle={this.close} className="MissionPlannerModal">
          <ModalHeader toggle={this.close}>Mission Planner</ModalHeader>
          <ModalBody>
            <div className="BigMissionPlanner">
              <MissionPlanner className="BigMissionPlanner"/>
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