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
        {this.props.missionPlan.commands.map((command) => 
          <div key={command.id}>
            Id: <span>{JSON.stringify(command)}</span>
          </div>
        )}
      </div>
    );
  }

  addCommand = () => {
    this.props.addWaypointCommand(null, this.props.protoInfo);
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(MissionPlanner);