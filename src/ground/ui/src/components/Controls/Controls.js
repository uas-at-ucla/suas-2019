import React, { Component } from 'react';

import './Controls.css';
import Map from './Map';
import Telemetry from './Telemetry/Telemetry';
import DroneActions from './DroneActions';
import MissionPlannerContainer from './MissionPlannerContainer/MissionPlannerContainer';

class Controls extends Component {
  render() {
    return (
      <div className="Controls">
        <Telemetry/>
        <DroneActions/>
        <MissionPlannerContainer/>
        <Map/>
      </div>
    );
  }
}

export default Controls;