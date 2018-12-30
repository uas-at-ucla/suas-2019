import React, { Component } from 'react';
import { Button } from 'reactstrap';
import MissionPlanner from './MissionPlanner'; 

class MissionPlannerContainer extends Component {
  render() {
    return (
      <div className="MissionPlannerContainer">
        <Button onClick={this.expand}>Expand</Button>
        <div className="MissionPlannerBig">
          <MissionPlanner></MissionPlanner>
        </div>
      </div>
    );
  }

  expand = () => {

  }
}

export default MissionPlannerContainer;