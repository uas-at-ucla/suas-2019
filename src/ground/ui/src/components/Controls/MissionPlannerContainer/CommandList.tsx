import React, { Component, PureComponent } from "react";
import { Button } from "reactstrap";
import { Row, Col, Input, InputGroup, InputGroupAddon } from "reactstrap";
import { SortableElement } from "react-sortable-hoc";
// import { CommandChangersType } from "./MissionPlanner";
import { CommandChangersType } from "./MissionPlanner";
// class MissionPlannerClass extends MissionPlanner.WrappedComponent {} // get class from redux wrapped component

interface CommandListProps {
  className: string;
  commands: any[];
  programType: string;
  mutable: boolean;
  protoInfo: any;
  centerMapOnCommand: (i: number) => void;
  commandChangers?: CommandChangersType; //MissionPlannerClass["commandChangers"];
}

interface CommandRowProps {
  className: string;
  key: string;
  command: any;
  programType: string;
  index: number;
  myIndex: number;
  mutable: boolean;
  protoInfo: any;
  centerMapOnCommand: (i: number) => void;
  commandChangers?: CommandChangersType; //MissionPlannerClass["commandChangers"];
}

const SortableCommand = SortableElement((props: CommandRowProps) => (
  <CommandRow {...props} />
));

class CommandList extends Component<CommandListProps> {
  public render() {
    let CommandRowElement = CommandRow;
    if (this.props.mutable) {
      CommandRowElement = SortableCommand as typeof CommandRow;
    }
    return (
      <span className="CommandList">
        {this.props.commands.map((command: any, index: number) => (
          <CommandRowElement
            className={this.props.className}
            key={command.id}
            centerMapOnCommand={this.props.centerMapOnCommand}
            commandChangers={this.props.commandChangers}
            command={command}
            programType={this.props.programType}
            index={index}
            myIndex={index}
            mutable={this.props.mutable}
            protoInfo={this.props.protoInfo}
          ></CommandRowElement>
        ))}
      </span>
    );
  }
}

export default CommandList;

// PureComponent improves performance b/c it only re-renders when props change
class CommandRow extends PureComponent<CommandRowProps> {
  private centerMapOnCommand = () => {
    console.log(this.props);
    this.props.centerMapOnCommand(this.props.myIndex);
  };

  public render() {
    let command = this.props.command;
    let index = this.props.myIndex;
    return (
      <Row
        className={`MissionPlanner ${this.props.className}`}
        onClick={this.centerMapOnCommand}
      >
        <Col xs="auto" className="command-column input command-header">
          <Button
            className="delete-btn"
            color="danger"
            size="sm"
            data-index={index}
            onClick={
              this.props.commandChangers &&
              this.props.commandChangers.deleteCommand
            }
          >
            <i className="fa fa-minus"></i>
          </Button>
        </Col>
        <Col xs="auto" className="command-column command-index command-header">
          {index + 1}
        </Col>
        <Col xs="auto" className="command-column command-type command-header">
          <span className="value">
            {this.props.protoInfo.commandAbbr[command.name]}
          </span>
          <Input
            type="select"
            className="input"
            value={command.name}
            readOnly={!this.props.mutable}
            data-index={index}
            onChange={
              this.props.commandChangers &&
              this.props.commandChangers.changeCommandType
            }
          >
            {this.props.protoInfo.commandNames.map((commandName: string) => (
              <option value={commandName} key={commandName}>
                {this.props.protoInfo.commandAbbr[commandName]}
              </option>
            ))}
          </Input>
        </Col>
        <Col xs="auto" className="command-column">
          <this.Field
            name=""
            dotProp={index + "." + command.name}
            type={command.type}
            object={command[command.name]}
          />
        </Col>
      </Row>
    );
  }

  // Helper components
  private NumberField = (props: {
    name: string;
    dotProp: string;
    value: number;
    units?: string;
  }) => {
    let { name, dotProp, value, units } = props;
    return (
      <Row>
        <Col xs="auto">
          <InputGroup className="number input">
            <InputGroupAddon addonType="prepend">{name}</InputGroupAddon>
            <Input
              style={{
                width: Math.min(12, value.toString().length + 3) + "ch"
              }}
              value={value}
              type="number"
              readOnly={!this.props.mutable}
              data-dot-prop={dotProp}
              onChange={
                this.props.commandChangers &&
                this.props.commandChangers.changeNumberField
              }
            ></Input>
            {units ? (
              <InputGroupAddon addonType="append">{units}</InputGroupAddon>
            ) : null}
          </InputGroup>
          <span className="value">
            {Math.round(value * 1e4) / 1e4} {units}
          </span>
        </Col>
      </Row>
    );
  };

  private RepeatedField = (props: {
    name: string;
    dotProp: string;
    type: string;
    object: any;
  }) => {
    let { name, dotProp, type, object } = props;
    return (
      <span>
        {object.map((element: any, index: number) => (
          <this.Field
            name={`${name} ${index + 1}`}
            key={index}
            dotProp={dotProp + "." + index}
            type={type}
            object={element}
          />
        ))}
        <Button
          className="input"
          data-dot-prop={dotProp}
          data-type={type}
          onClick={
            this.props.commandChangers &&
            this.props.commandChangers.addRepeatedField
          }
        >
          +
        </Button>
        <Button
          className="input"
          data-dot-prop={dotProp}
          onClick={
            this.props.commandChangers &&
            this.props.commandChangers.popRepeatedField
          }
        >
          -
        </Button>
      </span>
    );
  };

  private Field = (props: {
    name: string;
    dotProp: string;
    type: string;
    object: any;
  }) => {
    let { name, dotProp, type, object } = props;
    // Recursively create HTML based on protobuf definition
    let timelineGrammar = this.props.protoInfo.timelineGrammar;
    if (timelineGrammar[type]) {
      // object is a protobuf defined object
      return (
        <Row>
          {name != "" ? (
            <Col xs="auto" className="name">
              {name}:
            </Col>
          ) : null}
          {Object.keys(timelineGrammar[type].fields).map(fieldName => {
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
                {field.rule === "repeated" ? (
                  <this.RepeatedField {...fieldProps} />
                ) : field.rule === "required" ? (
                  <this.Field {...fieldProps} />
                ) : (
                  (() => {
                    throw new Error(
                      "No support for timeline_grammar rule '" +
                        field.rule +
                        "' yet!"
                    );
                  })()
                )}
              </Col>
            );
          })}
        </Row>
      );
    } else if (type === "double") {
      return (
        <this.NumberField
          name={name}
          dotProp={dotProp}
          value={object}
          units={this.props.protoInfo.fieldUnits[this.props.programType][name]}
        />
      );
    } else if (type === "bool") {
      return (
        <span style={{ color: object ? undefined : "dimgray" }}>{name}</span>
      ); // does not support modification because there are no bools in a GroundProgram
    } else {
      throw new Error(
        "No support for timeline_grammar type '" + type + "' yet!"
      );
    }
  };
}
