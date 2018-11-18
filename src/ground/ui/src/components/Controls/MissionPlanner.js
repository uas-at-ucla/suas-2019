import React, { Component } from 'react';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';

import missionActions from '../../actions/missionActions';
import { selector } from '../../store';

const mapStateToProps = state => { 
  return { 
    missionPlan: state.missionPlan,
    protoInfo: selector(state).missionPlan.protoInfo
  };
};

const mapDispatchToProps = missionActions;

class MissionPlanner extends Component {
  render() {
    return (
      <div className="MissionPlanner">
        <Button onClick={this.addCommand}>Add Command</Button>
        {this.props.missionPlan.commands.map((command, index) => 
          <div key={command.id}>
            <span>{`${index}: ${JSON.stringify(command)}`}</span>
          </div>
        )}
      </div>
    );
  }

  addCommand = () => {
    let defaultWaypointCommand = { goal: {
      latitude: 38.147483,
      longitude: -76.427778,
      altitude: 100
    }}
    this.props.addWaypointCommand(defaultWaypointCommand, this.props.protoInfo);
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(MissionPlanner);