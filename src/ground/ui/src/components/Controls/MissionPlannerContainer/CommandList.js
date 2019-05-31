import React, { Component, PureComponent } from 'react';
import { Button } from 'reactstrap';
import { Row, Col, Input, InputGroup, InputGroupAddon } from 'reactstrap';
import { SortableElement } from 'react-sortable-hoc';

class CommandList extends Component {
  render() {
    let CommandRowElement = CommandRow;
    if (this.props.mutable) {
      CommandRowElement = SortableCommand;
    }
    return (
      <span>
        {this.props.commands.map((command, index) => 
          <CommandRowElement
            className={this.props.className}
            key={command.id}
            {...this.props.commandChangers}
            command={command}
            index={index}
            myIndex={index}
            protoInfo={this.props.protoInfo}
          ></CommandRowElement>
        )}
      </span>
    );
  }
}

export default CommandList;

const SortableCommand = SortableElement(props => <CommandRow {...props}/>);
// PureComponent improves performance b/c it only re-renders when props change
class CommandRow extends PureComponent {
  centerMapOnCommand = () => {
    console.log(this.props);
    this.props.centerMapOnCommand(this.props.myIndex);
  }

  render() {
    let command = this.props.command;
    let index = this.props.myIndex;
    return (
      <Row className={`MissionPlanner ${this.props.className}`} onClick={this.centerMapOnCommand}>
        <Col xs="auto" className="command-column input command-header">
          <Button className="delete-btn" color="danger" size="sm" data-index={index} onClick={this.props.deleteCommand}>
            <i className="fa fa-minus"></i>
          </Button>
        </Col>
        <Col xs="auto" className="command-column command-index command-header">{index+1}</Col>
        <Col xs="auto" className="command-column command-type command-header">
          <span className="value">
            {this.props.protoInfo.commandAbbr[command.name]}
          </span>
          <Input
            type="select" className="input" value={command.name} readOnly={!this.props.mutable}
            data-index={index} onChange={this.props.changeCommandType}
          >
            {this.props.protoInfo.commandNames.map(commandName =>
              <option value={commandName} key={commandName}>
                {this.props.protoInfo.commandAbbr[commandName]}
              </option>
            )}
          </Input>
        </Col>
        <Col xs="auto" className="command-column">
          <this.Field
            dotProp={index + "." + command.name}
            type={command.type}
            object={command[command.name]}
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
              value={value} type="number" readOnly={!this.props.mutable}
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
    } else if (type === "bool") {
      return <span style={{color: object ? null : "dimgray"}}>{name}</span>; // does not support modification because there are no bools in a GroundProgram
    } else {
      throw new Error("No support for timeline_grammar type '" + type + "' yet!");
    }
  };
}
