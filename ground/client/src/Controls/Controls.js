import React, { Component } from 'react';
import './Controls.css';

class Controls extends Component {
  render() {
    return (
      <div id="controls" className="text-white">
        <div id="normalControls" className="card">
          <button className="btn btn-outline-success"
                  onClick={this.sendRunMissionCommand}>
            Run Mission
          </button>

          <button className="btn btn-outline-secondary"
                  onClick={this.sendLandCommand}>
            Land
          </button>
        </div>

        <div id="emergencyControls" className="card">
          <button className="btn btn-outline-warning"
                  onClick={this.sendFailsafeCommand}>
            Failsafe Land
          </button>

          <button className="btn btn-outline-danger"
                  onClick={this.sendThrottleCutCommand}>
            Throttle Cut
          </button>
        </div>
      </div>
    );
  }

  sendRunMissionCommand = () => {
    let commands = [];

    for (let command of this.props.homeState.commands) {
      commands.push({
        type: 'goto',
        pos: command.goto_options
      });
    }

    console.log(commands);

    this.props.socketEmit('execute_commands', commands);
  }

  sendLandCommand = () => {
    const command = {
      state: 'LAND'
    }

    this.props.socketEmit('set_state', command);
  }

  sendFailsafeCommand = () => {
    const command = {
      state: 'FAILSAFE'
    }

    this.props.socketEmit('set_state', command);
  }

  sendThrottleCutCommand = () => {
    const command = {
      state: 'THROTTLE_CUT'
    }

    this.props.socketEmit('set_state', command);
  }
}

export default Controls;
