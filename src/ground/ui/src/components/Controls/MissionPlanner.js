import React, { Component } from 'react';
import { Button } from 'reactstrap';
import missionActions from '../../actions/missionActions';

class MissionPlanner extends Component {
  state = {
    missionPlan: []
  }

  render() {
    return (
      <div className="MissionPlanner">
        <Button onClick={this.addCommand}>Add Command</Button>
        {this.state.missionPlan.map((command, index) => 
          <div key={index}>
            Name: <span>{command.name}</span>
          </div>
        )}
      </div>
    );
  }

  addCommand = () => {
    let newCommand = {
      name: "Name goes here"
    }
    this.setState({missionPlan: this.state.missionPlan.concat(newCommand)});
    console.log(this.state);
  }
}

export default MissionPlanner;