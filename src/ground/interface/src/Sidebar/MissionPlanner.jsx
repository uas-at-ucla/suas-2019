import React, { Component } from "react";
import { FormControl, FormGroup } from "react-bootstrap";
import {
  SortableContainer,
  SortableElement,
  arrayMove
} from "react-sortable-hoc";

const SortableItem = SortableElement(({ command, myIndex, self }) => (
  <tr
    className="command_row"
    onClick={event => self.onCommandClick(myIndex, event)}
  >
    <td>{myIndex + 1}</td>
    <td>
      <input
        className="name_input"
        type="text"
        value={command.name}
        onChange={event => self.onCommandNameChange(myIndex, event)}
      />
    </td>
    <td>
      <FormGroup controlId="formControlsSelect">
        <FormControl
          componentClass="select"
          placeholder="select"
          value={command.options.command_type}
          onChange={event => self.onCommandTypeChange(myIndex, event)}
        >
          <option>goto</option>
          <option>jump</option>
          <option>bomb</option>
          <option>survey</option>
        </FormControl>
      </FormGroup>
    </td>
    <td>
      <input
        className="altitude_input"
        type="number"
        value={command.options.alt}
        onChange={event => self.onCommandAltChange(myIndex, event)}
      />
    </td>
  </tr>
));

const SortableList = SortableContainer(({ commands, self }) => {
  return (
    <div className="scrollbar">
      <table id="commandList">
        <tbody>
          <tr className="command_row" id="commandListLabels">
            <td>#</td>
            <td>Name</td>
            <td>Type</td>
            <td>Alt</td>
          </tr>
          {commands.map((command, index) => (
            <SortableItem
              key={`item-${index}`}
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

  onCommandClick(index, event) {
    if (event.target.tagName === "TD") {
      this.props.setHomeState({ focusedCommand: index });
    }
  }
}

export default MissionPlanner;
