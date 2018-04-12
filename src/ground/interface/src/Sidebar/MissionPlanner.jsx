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
      <td>{String(Number(myIndex) + 1)}</td>
      <td>{command.name}</td>
      <td>
        <FormGroup className="type_select">
          {self.commandTypeOptions(myIndex, command)}
        </FormGroup>
      </td>
      {fields.map((field, index) => {
        return (
          <td key={index}>
            <InputGroup>
              <InputGroupAddon addonType="prepend">
                <InputGroupText>{field}</InputGroupText>
              </InputGroupAddon>
              <Input
                type="number"
                value={command[type][field]}
                className="command_input"
                onChange={new_value => {
                  self.changed_command_field(
                    new_value.target.value,
                    myIndex,
                    field
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
          <Button
            color="primary"
            className="mission_button"
            onClick={this.addCommand}
          >
            Add Command
          </Button>
        </div>
        {this.deleteBtn()}
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
      fields
    );
    if (commands[index].mission_point && command[newType].latitude) {
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

  deleteBtn() {
    if (this.state.deleteBtnIndex == null) {
      return null;
    }
    return (
      <button
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

  changed_command_field(value, command, field) {
    let commands = this.props.homeState.commands.slice();
    let type = commands[command].type;
    commands[command][type][field] = parseFloat(value);
    this.props.setHomeState({
      commands: commands,
      changedCommands: {startIndex: command, endIndex: command}
    });
  }
}

export default MissionPlanner;
