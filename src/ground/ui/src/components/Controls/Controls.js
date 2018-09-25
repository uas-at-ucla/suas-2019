import React, { Component } from 'react';

import './Controls.css';
import Map from './Map';
import Telemetry from './Telemetry';

class Controls extends Component {
  render() {
    return (
      <div className="Controls">
        <Telemetry/>
        <Map/>
      </div>
    );
  }
}

export default Controls;