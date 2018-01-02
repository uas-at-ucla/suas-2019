import React, { Component } from 'react';
import {
  FormControl,
  FormGroup,
} from 'react-bootstrap';

class MissionPlanner extends Component {

  render() {
    const commandsList = this.props.homeState.commands.map((command, index) =>
      <tr key={index}>
        <td>{index+1}</td>
        <td>
          <input
            className="name_input"
            type="text"
            value={command.name}
            onChange={
              event => this.onCommandNameChange(index, event)
            }>
          </input>
        </td>
        <td>
          <FormGroup controlId="formControlsSelect">
            <FormControl
              componentClass="select"
              placeholder="select"
              value={command.options.command_type}
              onChange={
                new_type => this.onCommandTypeChange(index, new_type)
              }>
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
            onChange={
              event => this.onCommandAltChange(index, event)
            }>
          </input>
        </td>
      </tr>
    );

    return (
      <div className="scrollbar">
        <table id="commandList">
          <tbody>
            <tr id="commandListLabels">
              <td>#</td>
              <td>Name</td>
              <td>Type</td>
              <td>Alt</td>
            </tr>
            {commandsList}
          </tbody>
        </table>
      </div>
    );
  }

  onCommandTypeChange(id, new_type) {
    let commands = this.props.homeState.commands.slice();
    commands[id].options.command_type = new_type.target.value;
    this.props.setHomeState({commands: commands});
  }

  onCommandNameChange(id, event) {
    let commands = this.props.homeState.commands.slice();
    commands[id].name = event.target.value;
    this.props.setHomeState({commands: commands});
  }

  onCommandAltChange(id, event) {
    let commands = this.props.homeState.commands.slice();
    commands[id].options.alt = event.target.value;
    this.props.setHomeState({commands: commands});
  }
}

export default MissionPlanner
