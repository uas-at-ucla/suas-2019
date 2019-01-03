import React, { Component, PureComponent } from 'react';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';
import { Container, Row, Col, Input, InputGroup, InputGroupAddon } from 'reactstrap';

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
            <CommandRow
              key={command.id}
              {...this.commandChangers}
              command={command}
              index={index}
              protoInfo={this.props.protoInfo}
            ></CommandRow>
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

  commandChangers = {
    changeCommandType: (event) => {
      let index = event.target.dataset.index;
      let newType = event.target.value;
      let oldCommand = this.props.missionPlan.commands[index];
      this.props.changeCommandType(index, oldCommand, newType, this.props.protoInfo);
    },
  
    changeNumberField: (event) => {
      let newValue = Number(event.target.value);
      if (!isNaN(newValue)) {
        let dotProp = event.target.dataset.dotProp;
        this.props.changeCommandField(dotProp, newValue);
      }
    },
  
    addRepeatedField: (event) => {
      let dotProp = event.target.dataset.dotProp;
      let type = event.target.dataset.type;
      this.props.addRepeatedField(dotProp, type, this.props.protoInfo);
    },
  
    popRepeatedField: (event) => {
      let dotProp = event.target.dataset.dotProp;
      this.props.popRepeatedField(dotProp);
    }
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(MissionPlanner);


// PureComponent improves performance b/c it only re-renders when props change
class CommandRow extends PureComponent {
  render() {
    let command = this.props.command;
    let index = this.props.index;
    return (
      <Row>
        <Col xs="auto" className="command-column command-index">{index+1}</Col>
        <Col xs="auto" className="command-column command-type">
          <span className="value">
            {this.props.protoInfo.commandAbbr[command.type]}
          </span>
          <Input
            type="select" className="input" value={command.type}
            data-index={index} onChange={this.props.changeCommandType}
          >
            {this.props.protoInfo.commandTypes.map(commandType =>
              <option value={commandType} key={commandType}>
              {this.props.protoInfo.commandAbbr[commandType]}
              </option>
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
    )
  }

  // Helper components
  NumberField = ({name, dotProp, value, units}) => {
    return (
      <Row>
        <Col xs="auto">
          <InputGroup className="number input">
            <InputGroupAddon addonType="prepend">{name}</InputGroupAddon>
            <Input
              style={{width: Math.min(12, value.toString().length + 3) + "ch"}}
              value={value} type="number"
              data-dot-prop={dotProp} onChange={this.props.changeNumberField}
            ></Input>
            {units ? <InputGroupAddon addonType="append">{units}</InputGroupAddon> : null}
          </InputGroup>
          <span className="value">{Math.round(value*1e4)/1e4} {units}</span>
        </Col>
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
          data-type={type} onClick={this.props.addRepeatedField}
        >+</Button>
        <Button
          className="input" data-dot-prop={dotProp}
          onClick={this.props.popRepeatedField}
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
      return <this.NumberField name={name} dotProp={dotProp} value={object} 
        units={this.props.protoInfo.fieldUnits[name]}/>;
    } else {
      throw new Error("No support for timeline_grammar type '" + type + "' yet!");
    }
  };
}
