import React, { Component } from 'react';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';
import { Container, Row, Col, Input } from 'reactstrap';

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
        <Container fluid>
          {this.props.missionPlan.commands.map((command, index) => 
            <Row key={command.id}>
              <Col xs="auto" className="command-column command-index">{index+1}</Col>
              <Col xs="auto" className="command-column command-type">
                <span className="value">{command.type}</span>
                <Input
                  type="select" className="input" value={command.type}
                  data-index={index} onChange={this.changeCommandType}
                >
                  {this.props.protoInfo.commandTypes.map(commandType =>
                    <option key={commandType}>{commandType}</option>
                  )}
                </Input>
              </Col>
              <Col xs="auto" className="command-column">
                <this.Field
                  dotProp={index + "." + command.type}
                  type={command.type}
                  object={command[command.type]}
                />
              </Col>
            </Row>
          )}
        </Container>
        <Button onClick={this.addCommand} className="command-btn">Add Command</Button>
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

  changeCommandType = (event) => {
    let index = event.target.dataset.index;
    let newType = event.target.value;
    let oldCommand = this.props.missionPlan.commands[index];
    this.props.changeCommandType(index, oldCommand, newType, this.props.protoInfo);
  }

  changeCommandField = (event) => {
    let dotProp = event.target.dataset.dotProp;
    let newValue = event.target.value;
    this.props.changeCommandField(dotProp, newValue);
  }

  addRepeatedField = (event) => {
    let dotProp = event.target.dataset.dotProp;
    let type = event.target.dataset.type;
    this.props.addRepeatedField(dotProp, type, this.props.protoInfo);
  }

  popRepeatedField = (event) => {
    let dotProp = event.target.dataset.dotProp;
    this.props.popRepeatedField(dotProp);
  }

  // Helper components
  fieldUnits = {
    altitude: "ft",
    dropHeight: "ft"
  }

  NumberField = ({name, dotProp, value, units}) => {
    return (
      <Row>
        <Col xs="auto" className="name">{name}:</Col>
        <Col xs="auto" className="number input">
          <Input
            value={value} type="number"
            data-dot-prop={dotProp} onChange={this.changeCommandField}
          ></Input>
        </Col>
        <Col xs="auto" className="value">{value}</Col>
        {units ? <Col xs="auto" className="units">{units}</Col> : null}
      </Row>
    );
  };

  RepeatedField = ({name, dotProp, type, object}) => {
    return (
      <span>
        {object.map((element, index) =>
          <this.Field
            name={`${name} ${index+1}`} key={index}
            dotProp={dotProp + "." + index}
            type={type}
            object={element}
          />
        )}
        <Button
          className="input" data-dot-prop={dotProp}
          data-type={type} onClick={this.addRepeatedField}
        >+</Button>
        <Button
          className="input" data-dot-prop={dotProp}
          onClick={this.popRepeatedField}
        >-</Button>
      </span>
    );
  }

  Field = ({name, dotProp, type, object}) => {
    // Recursively create HTML based on protobuf definition
    let timelineGrammar = this.props.protoInfo.timelineGrammar;
    if (timelineGrammar[type]) {
      // object is a protobuf defined object
      return (
        <Row>
          {name ? <Col xs="auto" className="name">{name}:</Col> : null}
          {Object.keys(timelineGrammar[type].fields).map((fieldName) => {
            let field = timelineGrammar[type].fields[fieldName];
            let fieldDotProp = dotProp + "." + fieldName;
            let fieldProps = {
              name: fieldName,
              dotProp: fieldDotProp,
              type: field.type,
              object: object[fieldName]
            };
            return (
              <Col xs="auto" className="field-container" key={fieldName}>
                {field.rule === 'repeated' ?
                  <this.RepeatedField {...fieldProps}/>
                : field.rule === 'required' ?
                  <this.Field {...fieldProps}/>
                :
                  (() => {throw new Error("No support for timeline_grammar rule '" + field.rule + "' yet!")})()
                }
              </Col>
            );
          })}
        </Row>
      );
    } else if (type === "double") {
      return <this.NumberField name={name} dotProp={dotProp} value={object} units={this.fieldUnits[name]}/>;
    } else {
      throw new Error("No support for timeline_grammar type '" + type + "' yet!");
    }
  };
}

export default connect(mapStateToProps, mapDispatchToProps)(MissionPlanner);
