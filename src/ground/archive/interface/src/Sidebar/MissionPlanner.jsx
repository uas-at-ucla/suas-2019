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

const SortableItem = SortableElement(({ command, changedCommands, newCmdStartEnd, myIndex, self }) => {
  let type = command.type;
  let fields = self.props.commandTypes[type];
  let startCmdIndex = self.props.homeState.commands.findIndex(cmd => cmd.isStart);
  let endCmdIndex = self.props.homeState.commands.findIndex(cmd => cmd.isEnd);
  if (endCmdIndex === -1) endCmdIndex = Infinity;
  if (startCmdIndex > endCmdIndex) {
    startCmdIndex = -1;
    endCmdIndex = Infinity;
  }

  return (
    <tr
      className="command_row"
      onClick={event => self.onCommandClick(myIndex, event)}
      key={myIndex}
      data-index={myIndex}
    >
      <td>
        <div
          style = {(startCmdIndex <= myIndex && myIndex <= endCmdIndex) ? {
            backgroundColor: "rgba(0, 255, 0, 0.25)"
          } : {}}
        >
          {String(Number(myIndex) + 1)}
        </div>
      </td>
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
    let result = nextProps.newCmdStartEnd || nextProps.changedCommands === null || (
        nextProps.changedCommands.startIndex <= nextProps.myIndex &&
        nextProps.myIndex <= nextProps.changedCommands.endIndex);
    return result;
  }
}

const SortableList = SortableContainer(({ commands, changedCommands, newCmdStartEnd, self }) => {
  return (
    <div className="scrollbar" onScroll={self.hideCmdPopup}>
      <table id="commandList">
        <tbody>
          {commands.map((command, index) => (
            <MySortableItem
              key={index}
              index={index}
              command={command}
              changedCommands={changedCommands}
              newCmdStartEnd={newCmdStartEnd}
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
    cmdPopupIndex: null,
    cmdPopupX: null,
    cmdPopupY: null
  }

  componentDidMount() {
    let modal = document.getElementById('missionPlanner').getElementsByClassName('modal-dialog')[0];

    modal.addEventListener('contextmenu', (e) => {
      let parentElement = e.target.parentElement;
      while (parentElement) {
        if (parentElement.className === 'command_row') {
          let index = Number(parentElement.getAttribute('data-index'));
          this.setState({
            cmdPopupIndex: index,
            cmdPopupX: e.clientX - modal.offsetLeft,
            cmdPopupY: e.clientY - modal.offsetTop
          });
          e.preventDefault();
          break;
        }
        parentElement = parentElement.parentElement;
      }
    });

    document.addEventListener('click', (e) => {
      this.hideCmdPopup();
    });
  }

  render() {
    return (
      <div>
        <SortableList
          commands={this.props.homeState.commands}
          changedCommands={this.props.homeState.changedCommands}
          newCmdStartEnd={this.props.homeState.newCmdStartEnd}
          self={this}
          onSortEnd={this.onSortEnd}
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
        {this.cmdPopup()}
      </div>
    );
  }

  addCommand = () => {
    let command = this.props.makeCommand('NothingCommand', null);
    let commands = this.props.homeState.commands;
    this.props.setHomeState({
      commands: commands.concat(command),
      changedCommands: {startIndex: commands.length, endIndex: commands.length}
    });
  }

  setCommandStart = (index) => {
    let commands = this.props.homeState.commands.slice();
    let startCmdIndex = commands.findIndex(cmd => cmd.isStart);
    if (startCmdIndex !== -1) {
      commands[startCmdIndex].isStart = false;
      this.props.setHomeState({
        commands: commands,
        changedCommands: {startIndex: startCmdIndex, endIndex: startCmdIndex}
      });
      commands = commands.slice();
    }
    let endCmdIndex = commands.findIndex(cmd => cmd.isEnd);
    if (endCmdIndex !== -1 && index > endCmdIndex) {
      commands[endCmdIndex].isEnd = false;
      this.props.setHomeState({
        commands: commands,
        changedCommands: {startIndex: endCmdIndex, endIndex: endCmdIndex}
      });
      commands = commands.slice();
    }
    commands[index].isStart = true;
    this.props.setHomeState({
      commands: commands,
      changedCommands: {startIndex: index, endIndex: index},
      newCmdStartEnd: true
    });
  }

  setCommandEnd = (index) => {
    let commands = this.props.homeState.commands.slice();
    let endCmdIndex = commands.findIndex(cmd => cmd.isEnd);
    if (endCmdIndex !== -1) {
      commands[endCmdIndex].isEnd = false;
      this.props.setHomeState({
        commands: commands,
        changedCommands: {startIndex: endCmdIndex, endIndex: endCmdIndex}
      });
      commands = commands.slice();
    }
    let startCmdIndex = commands.findIndex(cmd => cmd.isStart);
    if (startCmdIndex !== -1 && index < startCmdIndex) {
      commands[startCmdIndex].isStart = false;
      this.props.setHomeState({
        commands: commands,
        changedCommands: {startIndex: startCmdIndex, endIndex: startCmdIndex}
      });
      commands = commands.slice();
    }
    commands[index].isEnd = true;
    this.props.setHomeState({
      commands: commands,
      changedCommands: {startIndex: index, endIndex: index},
      newCmdStartEnd: true
    });
  }

  deleteCommand = (index) => {
    let commands = this.props.homeState.commands.slice();
    commands.splice(index, 1);
    this.props.setHomeState({
      commands: commands,
      changedCommands: {startIndex: index, endIndex: commands.length},
      newCmdStartEnd: true
    });
  }

  onSortEnd = ({ oldIndex, newIndex }) => {
    if (oldIndex !== newIndex) {
      this.hideCmdPopup();
      this.props.setHomeState({
        commands: arrayMove(this.props.homeState.commands, oldIndex, newIndex),
        changedCommands: {
          startIndex: Math.min(oldIndex, newIndex), 
          endIndex: Math.max(oldIndex, newIndex)
        },
        newCmdStartEnd: true
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
      command.isStart = commands[index].isStart;
      command.isEnd = commands[index].isEnd;
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
                <label className="form-check-label" htmlFor={index+field}>
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

  cmdPopup() {
    if (this.state.cmdPopupIndex == null) {
      return null;
    }
    return (
      <div
        id="cmd_popup"
        className="card"
        style={{
          position: 'fixed',
          left: this.state.cmdPopupX,
          top: this.state.cmdPopupY
        }}
      >
        <h6 class="card-title"><b>Command {this.state.cmdPopupIndex+1}</b></h6>
        <button
          className="btn btn-primary"
          onClick={() => this.setCommandStart(this.state.cmdPopupIndex)}
        >
          Set Start
        </button>
        <button
          className="btn btn-primary"
          onClick={() => this.setCommandEnd(this.state.cmdPopupIndex)}
        >
          Set End
        </button>
        <button
          className="btn btn-danger"
          onClick={() => this.deleteCommand(this.state.cmdPopupIndex)}
        >
          Delete
        </button>
      </div>
    );
  }

  hideCmdPopup = () => {
    if (this.state.cmdPopupIndex != null) {
      this.setState({cmdPopupIndex: null});
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
