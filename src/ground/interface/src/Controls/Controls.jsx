import React, { Component } from 'react';
import './Controls.css';
import PromptButton from '../PromptButton/PromptButton';

class Controls extends Component {

  keymap = {
    'KeyZ': 'run_mission_btn',
    'KeyX': 'pause_mission_btn',
    'KeyC': 'take_off_btn',
    'KeyV': 'hold_btn',
    'KeyB': 'offboard_btn',
    'KeyN': 'rtl_btn',
    'KeyM': 'land_btn',
    'Comma': 'alarm_btn',
    'Period': 'failsafe_btn',
    'Slash': 'throttle_cut_btn'
  }

  componentDidMount() {
    document.addEventListener("keypress", (e) => {
      if (e.ctrlKey && e.shiftKey && this.keymap[e.code]) {
        let button = this.refs[this.keymap[e.code]];
        button.click();
        if (button.tagName === 'BUTTON') {
          this.refs[this.keymap[e.code]].focus();
          console.log("click")
        }
      }
    });
  }

  render() {
    return (
      <div id="controls" className="text-white">
        <div id="normalControls" className="card">
          <button
            className="btn btn-outline-success"
            onClick={this.sendRunMissionCommand}
            id="run_mission_btn"
            ref="run_mission_btn"
          >
            Run Mission (Z)
          </button>

          <button
            className="btn btn-outline-secondary"
            onClick={() => this.setDroneState('PAUSE')}
            id="pause_mission_btn"
            ref="pause_mission_btn"
          >
            Pause (X)
          </button>
        </div>

        <div id="stateControls" className="card">
          <button
            className="btn btn-outline-success"
            onClick={() => this.setDroneState('TAKEOFF')}
            id="take_off_btn"
            ref="take_off_btn"
          >
            Take Off (C)
          </button>

          <button
            className="btn btn-outline-secondary"
            onClick={() => this.setDroneState('HOLD')}
            id="hold_btn"
            ref="hold_btn"
          >
            Hold (V)
          </button>


          <button
            className="btn btn-outline-primary"
            onClick={() => this.setDroneState('OFFBOARD')}
            id="offboard_btn"
            ref="offboard_btn"
          >
            Offboard (B)
          </button>

          <PromptButton
            className="btn btn-outline-warning"
            onClick={() => this.setDroneState('RTL')}
            id="rtl_btn"
            ref="rtl_btn"
          >
            RTL (N)
          </PromptButton>

          <PromptButton
            className="btn btn-outline-success"
            onClick={() => this.setDroneState('LAND')}
            id="land_btn"
            ref="land_btn"
          >
            Land (M)
          </PromptButton>

          <button
            className="btn btn-outline-danger"
            onClick={() => this.setDroneState('ALARM')}
            id="alarm_btn"
            ref="alarm_btn"
          >
            Alarm (&lt;)
          </button>

          <PromptButton
            className="btn btn-outline-warning"
            onClick={() => this.setDroneState('FAILSAFE')}
            id="failsafe_btn"
            ref="failsafe_btn"
          >
            Failsafe Land (&gt;)
          </PromptButton>

          <PromptButton
            className="btn btn-outline-danger"
            onClick={() => this.setDroneState('THROTTLE CUT')}
            id="throttle_cut_btn"
            ref="throttle_cut_btn"
          >
            Throttle Cut (?)
          </PromptButton>
        </div>
      </div>
    );
  }

  setDroneState(state) {
    const command = {
      state: state
    };

    this.props.socketEmit('set_state', command);
  }

  sendRunMissionCommand = () => {
    const serialized_mission = this.props.getMission();

    this.props.socketEmit(
      'execute_commands',
      this.bin2string(serialized_mission)
    );
  };

  bin2string(arrayBuffer) {
    // Taken from here: https://gist.github.com/jonleighton/958841
    var base64 = '';
    var encodings =
      'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';

    var bytes = new Uint8Array(arrayBuffer);
    var byteLength = bytes.byteLength;
    var byteRemainder = byteLength % 3;
    var mainLength = byteLength - byteRemainder;

    var a, b, c, d;
    var chunk;

    // Main loop deals with bytes in chunks of 3
    for (var i = 0; i < mainLength; i = i + 3) {
      // Combine the three bytes into a single integer
      chunk = (bytes[i] << 16) | (bytes[i + 1] << 8) | bytes[i + 2];

      // Use bitmasks to extract 6-bit segments from the triplet
      a = (chunk & 16515072) >> 18; // 16515072 = (2^6 - 1) << 18
      b = (chunk & 258048) >> 12; // 258048   = (2^6 - 1) << 12
      c = (chunk & 4032) >> 6; // 4032     = (2^6 - 1) << 6
      d = chunk & 63; // 63       = 2^6 - 1

      // Convert the raw binary segments to the appropriate ASCII encoding
      base64 += encodings[a] + encodings[b] + encodings[c] + encodings[d];
    }

    // Deal with the remaining bytes and padding
    if (byteRemainder == 1) {
      chunk = bytes[mainLength];

      a = (chunk & 252) >> 2; // 252 = (2^6 - 1) << 2

      // Set the 4 least significant bits to zero
      b = (chunk & 3) << 4; // 3   = 2^2 - 1

      base64 += encodings[a] + encodings[b] + '==';
    } else if (byteRemainder == 2) {
      chunk = (bytes[mainLength] << 8) | bytes[mainLength + 1];

      a = (chunk & 64512) >> 10; // 64512 = (2^6 - 1) << 10
      b = (chunk & 1008) >> 4; // 1008  = (2^6 - 1) << 4

      // Set the 2 least significant bits to zero
      c = (chunk & 15) << 2; // 15    = 2^4 - 1

      base64 += encodings[a] + encodings[b] + encodings[c] + '=';
    }

    return base64;
  }
}

export default Controls;
