import React, { Component } from 'react';
import {
  FormControl,
  FieldGroup,
  FormGroup,
  ControlLabel
} from 'react-bootstrap';
import {
  InputGroup,
  InputGroupAddon,
  InputGroupText,
  InputGroupButton,
  Input,
  Button
} from 'reactstrap';

import {
  SortableContainer,
  SortableElement,
  arrayMove
} from 'react-sortable-hoc';

const SortableItem = SortableElement(({ command, myIndex, self }) => {
  let type = command.type;
  let fields = Object.keys(command[type]);

  if (self.state.command_field[myIndex] == null) {
    self.state.command_field[myIndex] = {};
  }

  return (
    <tr
      className="command_row"
      onClick={event => self.onCommandClick(myIndex, event)}
      key={myIndex}
    >
      <td>{String(Number(myIndex) + 1)}</td>
      <td>{command.name}</td>
      <td>
        <FormGroup className="type_select">
          {self.commandTypeOptions(myIndex, command)}
        </FormGroup>
      </td>
      {fields.map((field, index) => {
        if (self.state.command_field[myIndex][index] == null) {
          self.state.command_field[myIndex][index] = command[type][field];
        }
        return (
          <td key={index}>
            <InputGroup>
              <InputGroupAddon addonType="prepend">
                <InputGroupText>{field}</InputGroupText>
              </InputGroupAddon>
              <Input
                value={self.state.command_field[myIndex][index]}
                className="command_input"
                onChange={new_value => {
                  self.changed_command_field(
                    new_value.target.value,
                    myIndex,
                    index
                  );
                }}
                bsSize="small"
                disabled={command.mission_point && (field === 'latitude' || field === 'longitude')}
              />
            </InputGroup>
          </td>
        );
      })}
    </tr>
  );
});

const SortableList = SortableContainer(({ commands, self }) => {
  return (
    <div className="scrollbar">
      <table id="commandList">
        <tbody>
          {commands.map((command, index) => (
            <SortableItem
              key={index}
              index={index}
              command={command}
              myIndex={index}
              self={self}
            />
          ))}
        </tbody>
      </table>
    </div>
  );
});

class MissionPlanner extends Component {
  state = {
    command_field: {},
    can_apply_modifications: true,
    command_field_reset: true
  };

  componentWillReceiveProps(nextProps) {
    if (nextProps.homeState.commands !== this.props.homeState.commands) {
      if(this.state.command_field_reset) {
        this.setState({command_field: {}});
      }
    }
  }

  render() {
    return (
      <div>
        <SortableList
          commands={this.props.homeState.commands}
          onSortEnd={this.onSortEnd}
          self={this}
          transitionDuration={200}
          distance={2}
        />
        <div className="add_command_button_wrapper">
          <Button
            color="primary"
            className="mission_button"
            onClick={() =>
              this.props.addCommand('NothingCommand', null)
            }
          >
            Add Command
          </Button>
          <br />
          <Button
            color={
              this.state.command_field_reset
                ? 'secondary'
                : this.state.can_apply_modifications ? 'success' : 'danger'
            }
            disabled={
              !this.state.can_apply_modifications ||
              this.state.command_field_reset
            }
            className="mission_button"
            onClick={() => this.apply_changes()}
          >
            {this.state.command_field_reset
              ? 'No Changes'
              : 'Apply Modifications'}
          </Button>
          <Button
            color="secondary"
            className="mission_button"
            color={this.state.command_field_reset ? "secondary" : "warning"}
            disabled={this.state.command_field_reset}
            onClick={() => this.reset_changes()}
          >
            Reset Changes
          </Button>
        </div>
      </div>
    );
  }

  onSortEnd = ({ oldIndex, newIndex }) => {
    if (oldIndex !== newIndex) {
      this.props.setHomeState({
        commands: arrayMove(this.props.homeState.commands, oldIndex, newIndex)
      });
    }
  };

  onCommandTypeChange(index, event) {
    let commands = this.props.homeState.commands.slice();
    let type = commands[index].type;
    let fields = commands[index][type];
    let newType = event.target.value;
    let command = this.props.makeCommand(
      newType,
      fields
    );
    if (commands[index].mission_point && command[newType].latitude) {
      delete commands[index][type];
      commands[index].type = newType;
      commands[index][newType] = command[newType];
    } else {
      commands[index] = command;
    }
    
    this.props.setHomeState({ commands: commands });
  }

  onCommandClick(index, event) {
    if (event.target.tagName === 'TD') {
      this.props.setHomeState({ focusedCommand: index });
    }
  }

  commandTypeOptions(index, command) {
    let items = Object.keys(this.props.commandTypes);

    return (
      <FormControl
        componentClass="select"
        placeholder="select"
        value={command.type}
        onChange={event => this.onCommandTypeChange(index, event)}
        key={index}
      >
        {items.map((item, index) => <option key={index}>{item}</option>)};
      </FormControl>
    );
  }

  changed_command_field(value, command, index) {
    let new_command_field = {...this.state.command_field};
    new_command_field[command][index] = value;
    this.setState({command_field_reset: false, command_field: new_command_field});

    this.check_can_apply();
  }

  check_can_apply() {
    for (let command of Object.keys(this.state.command_field)) {
      for (let index of Object.keys(this.state.command_field[command])) {
        if (isNaN(this.state.command_field[command][index])) {
          this.setState({ can_apply_modifications: false });
          return;
        }
      }
    }

    this.setState({ can_apply_modifications: true });
    return;
  }

  apply_changes() {
    let cmds_copy = this.props.homeState.commands.slice();
    for (let command of Object.keys(this.state.command_field)) {
      for (let index of Object.keys(this.state.command_field[command])) {
        let cmd_type = this.props.homeState.commands[command].type;
        let field = Object.keys(
          this.props.homeState.commands[command][cmd_type]
        )[index];

        cmds_copy[command][cmd_type][field] = parseFloat(
          this.state.command_field[command][index]
        );
      }
    }

    this.props.setHomeState({ commands: cmds_copy });
    this.setState({ command_field_reset: true });
  }

  reset_changes() {
    this.setState({ command_field: {}, command_field_reset: true });
  }
}

export default MissionPlanner;
