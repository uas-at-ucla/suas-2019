import React, { Component } from 'react';
import {
  FormControl,
  FormGroup,
} from 'react-bootstrap';

class MissionPlanner extends Component {
  constructor(props) {
    super(props);

    this.state = {
      totalCommands: 0
    }
  }

  render() {
    const commandsList = this.props.homeState.commands.map((command) =>
      <tr key={command.id}>
        <td>{command.id}</td>
        <td>
          <FormGroup controlId="formControlsSelect">
            <FormControl
              componentClass="select"
              placeholder="select"
              onChange={
                new_type => this.onCommandTypeChange(command.id, new_type)
              }>
              <option>goto</option>
              <option>jump</option>
              <option>bomb</option>
              <option>survey</option>
            </FormControl>
          </FormGroup>
        </td>
      </tr>
    );

    return (
      <div className="scrollbar">
        <table id="commandList">
          <tbody>
            <tr id="commandListLabels">
              <td>ID</td>
              <td>Type</td>
            </tr>
            {commandsList}
          </tbody>
        </table>
      </div>
    );
  }

  onCommandTypeChange(id, new_type) {
    this.props.homeState.commands[id].type = new_type.target.value;

    // Force a redraw.
    this.setState(this.state);
  }

  addGotoCommand(lat, lng) {
    let commands = this.props.homeState.commands.slice();

    let goto_command = {
      id: this.state.totalCommands,
      type: "goto",
      goto_options: {
        lat: lat,
        lng: lng,
        alt: 80,
      }
    }

    commands.push(goto_command);

    this.props.setHomeState({commands: commands});
    this.state.totalCommands++;
  }
}

export default MissionPlanner
