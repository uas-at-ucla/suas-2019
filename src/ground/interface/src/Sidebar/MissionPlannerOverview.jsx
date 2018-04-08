import React, { Component } from 'react';
import {
  FormControl,
  FieldGroup,
  FormGroup,
  ControlLabel
} from 'react-bootstrap';
import { InputGroup, InputGroupAddon, InputGroupText, Input } from 'reactstrap';

import {
  SortableContainer,
  SortableElement,
  arrayMove
} from 'react-sortable-hoc';
import './MissionPlanner.css';

const SortableItem = SortableElement(({ command, myIndex, self }) => {
  let type = Object.keys(command)[0];
  let fields = Object.keys(command[type]);

  return (
    <tr
      className="command_row"
      onClick={event => self.onCommandClick(myIndex, event)}
    >
      <td>{myIndex + 1}</td>
      <td>{type}</td>
      {fields.map((field, index) => (
        <td key={index}>{parseFloat(command[type][field]).toFixed(3)}</td>
      ))}
    </tr>
  );
});

const SortableList = SortableContainer(({ commands, self }) => {
  return (
    <div className="scrollbar">
      <table id="commandListOverview">
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

class MissionPlannerOverview extends Component {
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
    commands[index].options.command_type = event.target.value;
    this.props.setHomeState({
      commands: commands,
      dontRedrawCommands: true
    });
  }

  onCommandNameChange(index, event) {
    let commands = this.props.homeState.commands.slice();
    commands[index].name = event.target.value;
    this.props.setHomeState({
      commands: commands,
      dontRedrawCommands: true
    });
  }

  onCommandAltChange(index, event) {
    let commands = this.props.homeState.commands.slice();
    commands[index].options.alt = Number(event.target.value);
    this.props.setHomeState({
      commands: commands,
      dontRedrawCommands: true
    });
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
}

export default MissionPlannerOverview;
