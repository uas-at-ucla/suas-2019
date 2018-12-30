import React, { Component } from 'react';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';

import missionActions from '../../../actions/missionActions';
import { selector } from '../../../store';

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
            <span>{index+1}: </span>
            <span>{command.type}</span>
            <Field
              type={command.type}
              object={command[command.type]}
              protoInfo={this.props.protoInfo}
            />
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


// Helper components
const fieldUnits = {
  altitude: "ft",
  dropHeight: "ft"
}

function NumberField({name, value, units}) {
  return (
    <div>
      <span className="name">{name}: </span>
      <input className="input" value={value} type="number"></input>
      <span className="value">{value}</span>
      <span className="units"> {units}</span>
    </div>
  );
};

function RepeatedField({name, type, object, protoInfo}) {
  return (
    <div>
      <span className="name">{name}</span>
      {object.map((element, index) =>
        <Field
          name={index+1} key={index}
          type={type}
          object={element}
          protoInfo={protoInfo}
        />
      )}
      <button class="add-repeated">Add</button>
    </div>
  );
}

function Field({name, type, object, protoInfo}) {
  // Recursively create HTML based on protobuf definition
  if (protoInfo.timelineGrammar[type]) {
    // object is a protobuf defined object
    return (
      <div>
        <span className="name">{name}: </span>
        {Object.keys(protoInfo.timelineGrammar[type].fields).map((fieldName) => {
          let field = protoInfo.timelineGrammar[type].fields[fieldName];
          let fieldProps = {
            key: fieldName,
            name: fieldName,
            type: field.type,
            object: object[fieldName],
            protoInfo: protoInfo
          };
          if (field.rule === 'repeated') {
            return <RepeatedField {...fieldProps}/>
          } else if (field.rule === 'required') {
            return <Field {...fieldProps}/>
          } else {
            throw new Error("No support for timeline_grammar rule '" + field.rule + "' yet!");
          }
        })}
      </div>
    );
  } else if (type === "double") {
    return <NumberField name={name} value={object} units={fieldUnits[name]}/>;
  } else {
    throw new Error("No support for timeline_grammar type '" + type + "' yet!");
  }
};
