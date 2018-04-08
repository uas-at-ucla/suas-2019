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
  let type = Object.keys(command)[0];
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
      <td>
        <FormGroup className="type_select" style={{ width: '150px' }}>
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
      <div className="add_command_button_wrapper">
        <Button
          color="primary"
          className="mission_button"
          onClick={() =>
            self.props.homeState.add_command('NothingCommand', null)
          }
        >
          Add Command
        </Button>
        <br />
        <Button
          color={
            self.state.command_field_reset
              ? 'secondary'
              : self.state.can_apply_modifications ? 'success' : 'danger'
          }
          disabled={
            !self.state.can_apply_modifications ||
            self.state.command_field_reset
          }
          className="mission_button"
          onClick={() => self.apply_changes()}
        >
          {self.state.command_field_reset
            ? 'No Changes'
            : 'Apply Modifications'}
        </Button>
        <Button
          color="secondary"
          className="mission_button"
          color={self.state.command_field_reset ? "secondary" : "warning"}
          disabled={self.state.command_field_reset}
          onClick={() => self.reset_changes()}
        >
          Reset Changes
        </Button>
      </div>
    </div>
  );
});

class MissionPlanner extends Component {
  state = {
    command_field: {},
    current_commands: null,
    can_apply_modifications: true,
    command_field_reset: true
  };

  componentWillReceiveProps(nextProps) {
    if (
      JSON.stringify(nextProps.homeState.commands) !=
      this.state.current_commands
    ) {
      if(this.state.command_field_reset) {
        this.setState({command_field: {}});
        this.state.current_commands = JSON.stringify(
          nextProps.homeState.commands
        );
      }
    }
  }

  render() {
    return (
      <SortableList
        commands={this.props.homeState.commands}
        onSortEnd={this.onSortEnd}
        self={this}
        transitionDuration={200}
        distance={2}
      />
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
    commands[index] = this.props.homeState.make_command(
      event.target.value,
      null
    );
    this.props.setHomeState({ commands: commands });
  }

  onCommandNameChange(index, event) {
    let commands = this.props.homeState.commands.slice();
    commands[index].name = event.target.value;
    this.props.setHomeState({ commands: commands });
  }

  onCommandAltChange(index, event) {
    let commands = this.props.homeState.commands.slice();
    commands[index].options.alt = Number(event.target.value);
    this.props.setHomeState({ commands: commands });
  }

  onLineSepChange(index, event) {
    let commands = this.props.homeState.commands.slice();
    commands[index].options.line_sep = Number(event.target.value);
    this.props.setHomeState({ commands: commands });
  }

  onCommandClick(index, event) {
    if (event.target.tagName === 'TD') {
      this.props.setHomeState({ focusedCommand: index });
    }
  }

  commandTypeOptions(index, command) {
    let items = Object.keys(this.props.homeState.command_types);

    return (
      <FormControl
        componentClass="select"
        placeholder="select"
        defaultValue={Object.keys(command)[0]}
        onChange={event => this.onCommandTypeChange(index, event)}
        key={index}
      >
        {items.map((item, index) => <option key={index}>{item}</option>)};
      </FormControl>
    );
  }

  changed_command_field(value, command, index) {
    this.state.command_field[command][index] = value;
    this.setState({ command_field_reset: false });

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
        let cmd_type = Object.keys(this.props.homeState.commands[command])[0];
        let field = Object.keys(
          this.props.homeState.commands[command][cmd_type]
        )[index];

        cmds_copy[command][cmd_type][field] = parseFloat(
          this.state.command_field[command][index]
        );
      }
    }

    this.props.homeState.set_home_state({ commands: cmds_copy });
    this.setState({ command_field_reset: true });
  }

  reset_changes() {
    this.setState({ command_field: {}, command_field_reset: true });
  }
}

export default MissionPlanner;
