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

const METERS_PER_FOOT = 0.3048;

const SortableItem = SortableElement(({ command, changedCommands, myIndex, self }) => {
  let type = command.type;
  let fields = self.props.commandTypes[type];

  return (
    <tr
      className="command_row"
      onClick={event => self.onCommandClick(myIndex, event)}
      key={myIndex}
      data-index={myIndex}
    >
      <td><div>{String(Number(myIndex) + 1)}</div></td>
      <td><div>{command.name}</div></td>
      <td><div>
        <FormGroup className="type_select">
          {self.commandTypeOptions(myIndex, command)}
        </FormGroup>
      </div></td>
      {fields.map((field, index) => {
        return self.commandFieldInput(myIndex, command, field.name, index);
      })}
    </tr>
  );
});

class MySortableItem extends SortableItem {
  shouldComponentUpdate(nextProps, nextState) {
    let result = nextProps.changedCommands === null || (
        nextProps.changedCommands.startIndex <= nextProps.myIndex &&
        nextProps.myIndex <= nextProps.changedCommands.endIndex);
    return result;
  }
}

const SortableList = SortableContainer(({ commands, changedCommands, self }) => {
  return (
    <div className="scrollbar" onScroll={self.hideDeleteBtn}>
      <table id="commandList">
        <tbody>
          {commands.map((command, index) => (
            <MySortableItem
              key={index}
              index={index}
              command={command}
              changedCommands={changedCommands}
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
    deleteBtnIndex: null,
    deleteBtnX: null,
    deleteBtnY: null
  }

  componentDidMount() {
    let modal = document.getElementById('missionPlanner').getElementsByClassName('modal-dialog')[0];

    modal.addEventListener('contextmenu', (e) => {
      let parentElement = e.target.parentElement;
      while (parentElement) {
        if (parentElement.className === 'command_row') {
          let index = parentElement.getAttribute('data-index');
          this.setState({
            deleteBtnIndex: index,
            deleteBtnX: e.clientX - modal.offsetLeft,
            deleteBtnY: e.clientY - modal.offsetTop
          });
          e.preventDefault();
          break;
        }
        parentElement = parentElement.parentElement;
      }
    });

    document.addEventListener('click', (e) => {
      this.hideDeleteBtn();
    });
  }

  render() {
    return (
      <div>
        <SortableList
          commands={this.props.homeState.commands}
          changedCommands={this.props.homeState.changedCommands}
          onSortEnd={this.onSortEnd}
          self={this}
          transitionDuration={200}
          distance={2}
        />
        <div className="add_command_button_wrapper">
          {this.props.homeState.invalidCommands ? 
            <Button
              color="success"
              className="mission_button"
              onClick={this.restoreCommands}
            >
              Restore Commands
            </Button> : 
            <Button
              color="primary"
              className="mission_button"
              onClick={this.addCommand}
            >
              Add Command
            </Button>
          }
          <Button
              color="danger"
              className="mission_button"
              onClick={this.clearCommands}
            >
              {this.state.willClear ? "Confirm" : "Clear Commands"}
          </Button>
        </div>
        {this.deleteBtn()}
      </div>
    );
  }

  addCommand = () => {
    let command = this.props.makeCommand('NothingCommand', null);
    let commands = this.props.homeState.commands;
    console.log(commands)
    this.props.setHomeState({
      commands: commands.concat(command),
      changedCommands: {startIndex: commands.length, endIndex: commands.length}
    });
  }


  deleteCommand = (index) => {
    let commands = this.props.homeState.commands.slice();
    commands.splice(index, 1);
    this.props.setHomeState({
      commands: commands,
      changedCommands: {startIndex: index, endIndex: commands.length}
    });
  }

  onSortEnd = ({ oldIndex, newIndex }) => {
    if (oldIndex !== newIndex) {
      this.hideDeleteBtn();
      this.props.setHomeState({
        commands: arrayMove(this.props.homeState.commands, oldIndex, newIndex),
        changedCommands: {
          startIndex: Math.min(oldIndex, newIndex), 
          endIndex: Math.max(oldIndex, newIndex)
        }
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
      fields,
      type
    );
    if (commands[index].mission_point && this.props.getCommandPosKey(newType)) {
      delete commands[index][type];
      commands[index].type = newType;
      commands[index][newType] = command[newType];
    } else {
      commands[index] = command;
    }
    
    this.props.setHomeState({
      commands: commands,
      changedCommands: {startIndex: index, endIndex: index}
    });
  }

  onCommandClick(index, event) {
    if (event.target.tagName === 'TD' || event.target.tagName === 'DIV'
        || event.target.tagName === 'B') {
      this.props.setHomeState({ focusedCommand: index });
    }
  }

  commandFieldInput(index, command, field, id) {
    let type = command.type;
    let keys = field.split('.');
    let subcommand = command[type];
    let key = null;
    for (key of keys) {
      if (Array.isArray(subcommand)) {
        subcommand = subcommand[parseInt(key)];
      } else {
        type = this.props.commandTypes[type].find(el => el.name === key).type;
        subcommand = subcommand[key];
      }
      if (subcommand == null) {
        return null;
      }
    }
    if (typeof subcommand !== 'object') {
      if (type === 'bool') {
        return (
          <td key={id} className="input_td">
            <div>
              <div className="form-check">
                <input
                  className="form-check-input"
                  type="checkbox"
                  checked={subcommand}
                  onChange={new_value => {
                    this.changed_bool_field(
                      new_value.target.checked,
                      index,
                      field
                    );
                  }}
                  id={index+field}
                />
                <label className="form-check-label" for={index+field}>
                  {key}
                </label>
              </div>
            </div>
          </td>
        );
      }
      return (
        <td key={id} className="input_td">
          {this.possibleUnits(key).map(units =>
          <div className={units}>
          <InputGroup>
            <InputGroupAddon addonType="prepend">
              <InputGroupText>{key}</InputGroupText>
            </InputGroupAddon>
            <Input
              type="number"
              value={this.convertToUnitsIfDistance(units, subcommand)}
              className="command_input"
              onChange={new_value => {
                this.changed_number_field(
                  this.convertToMetersIfDistance(units, new_value.target.value),
                  index,
                  field
                );
              }}
              bsSize="small"
              disabled={command.interop_object && (key === 'latitude' || key === 'longitude')}
            />
            {units !== "none" ? 
              <InputGroupAddon addonType="append">
                <InputGroupText>{units==="metric" ? "m" : "ft"}</InputGroupText>
              </InputGroupAddon> : null
            }
          </InputGroup>
          </div>)}
        </td>
      );
    } else {
      let fields = this.props.commandTypes[type];
      if (Array.isArray(subcommand)) {
        return (
          <td key={id}>
            <div>
            <table>
              <tbody>
                {subcommand.map((cmd, i) => 
                  <tr key={i}>
                    <td><b>{key} {i+1}:</b></td>
                    {fields.map((next_field, j) => {
                      return this.commandFieldInput(index, command, field+'.'+i+'.'+next_field.name, j);
                    })}
                  </tr>
                )}
              </tbody>
            </table>
            </div>
          </td>
        );
      }
      return (
        <td key={id}>
          <div>
          <table>
            <tbody>
              <tr>
                <td><b>{key}:</b></td>
                {fields.map((next_field, i) => {
                  return this.commandFieldInput(index, command, field+'.'+next_field.name, i);
                })}
              </tr>
            </tbody>
          </table>
          </div>
        </td>
      );
    }
  }

  possibleUnits(value_type) {
    if (value_type === "altitude") {
      return ["metric", "imperial"];
    }
    return ["none"];
  }

  convertToMetersIfDistance(fromUnits, value) {
    if (fromUnits === "imperial") {
      return value * METERS_PER_FOOT;
    }
    return value;
  }

  convertToUnitsIfDistance(toUnits, value) {
    if (toUnits === "imperial") {
      return this.round(value / METERS_PER_FOOT, 3);
    } else if (toUnits === "metric") {
      return this.round(value, 3);
    }
    return value;
  }

  commandTypeOptions(index, command) {
    let items = this.props.commandTypes['Command'].map(el => el.type);

    return (
      <FormControl
        componentClass="select"
        placeholder="select"
        value={command.type}
        onChange={event => this.onCommandTypeChange(index, event)}
        key={index}
      >
        {items.map((item, index) => <option key={index} value={item}>{item.replace('Command', '')}</option>)};
      </FormControl>
    );
  }

  deleteBtn() {
    if (this.state.deleteBtnIndex == null) {
      return null;
    }
    return (
      <button
        id="delete_command_btn"
        className="btn btn-danger"
        style={{
          position: 'fixed',
          left: this.state.deleteBtnX,
          top: this.state.deleteBtnY
        }}
        onClick={() => this.deleteCommand(this.state.deleteBtnIndex)}
      >
        Delete Command {Number(this.state.deleteBtnIndex)+1}
      </button>
    );
  }

  hideDeleteBtn = () => {
    if (this.state.deleteBtnIndex != null) {
      this.setState({deleteBtnIndex: null});
    }
  }

  changed_number_field(value, command, field) {
    if (value === '') value = 0;
    let newValue = parseFloat(value);
    if (!isNaN(newValue)) {
      this.changed_command_field(newValue, command, field);
    }
  }

  changed_bool_field(value, command, field) {
    this.changed_command_field(value, command, field);
  }

  changed_command_field(value, command, field) {
    let commands = this.props.homeState.commands.slice();
    let type = commands[command].type;

    let keys = field.split('.');
    field = keys.pop();
    let subcommand = commands[command][type];
    for (let key of keys) {
      if (Array.isArray(subcommand)) {
        subcommand = subcommand[parseInt(key)];
      } else {
        subcommand = subcommand[key];
      }
    }

    if (Array.isArray(subcommand)) {
      subcommand[parseInt(field)] = value;
    } else {
      subcommand[field] = value;
    }

    this.props.setHomeState({
      commands: commands,
      changedCommands: {startIndex: command, endIndex: command}
    });
  }

  restoreCommands = () => {
    this.props.socketEmit('request_commands', {restoreCommands: true});
  }

  clearCommands = () => {
    if (this.state.willClear) {
      this.props.setHomeState({
        commands: [],
        changedCommands: null,
        invalidCommands: false
      });
      this.setState({willClear: false});
    } else {
      this.setState({willClear: true});
      setTimeout(() => this.setState({willClear: false}), 2000);
    }
  }

  round(value, precision) {
    var multiplier = Math.pow(10, precision || 0);
    return Math.round(value * multiplier) / multiplier;
  }
}

export default MissionPlanner;
