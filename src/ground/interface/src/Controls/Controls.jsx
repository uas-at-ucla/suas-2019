import React, { Component } from 'react';
import './Controls.css';
import PromptButton from '../PromptButton/PromptButton';

class Controls extends Component {
  render() {
    return (
      <div id="controls" className="text-white">
        <div id="normalControls" className="card">
          <PromptButton
            className="btn btn-outline-success"
            onClick={this.sendRunMissionCommand}
            id="run_mission_btn"
          >
            Run Mission
          </PromptButton>

          <PromptButton
            className="btn btn-outline-secondary"
            onClick={this.sendLandCommand}
            id="land_btn"
          >
            Land
          </PromptButton>
        </div>

        <div id="emergencyControls" className="card">
          <PromptButton
            className="btn btn-outline-warning"
            onClick={this.sendFailsafeCommand}
            id="failsafe_btn"
          >
            Failsafe Land
          </PromptButton>

          <PromptButton
            className="btn btn-outline-danger"
            onClick={this.sendThrottleCutCommand}
            id="throttle_cut_btn"
          >
            Throttle Cut
          </PromptButton>
        </div>
      </div>
    );
  }

  sendRunMissionCommand = () => {
    const serialized_mission = this.props.getMission();

    this.props.socketEmit(
      'execute_commands',
      this.bin2string(serialized_mission)
    );
  };

  sendLandCommand = () => {
    const command = {
      state: 'LAND'
    };

    this.props.socketEmit('set_state', command);
  };

  sendFailsafeCommand = () => {
    const command = {
      state: 'FAILSAFE'
    };

    this.props.socketEmit('set_state', command);
  };

  sendThrottleCutCommand = () => {
    const command = {
      state: 'THROTTLE CUT'
    };

    this.props.socketEmit('set_state', command);
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
