import React, { Component } from 'react';

import './Controls.css';
import Map from './Map';
import Telemetry from './Telemetry/Telemetry';
import DroneActions from './DroneActions';
import MissionPlannerContainer from './MissionPlannerContainer/MissionPlannerContainer';
import Cosmetics from '../Utils/Cosmetics/Cosmetics';

class Controls extends Component {
  render() {
    return (
      <div className="Controls">
        <div className="map-overlay">
          <div>
            <div className="left-side">
              <div>
                <Cosmetics/>
                <MissionPlannerContainer/>
                <DroneActions/>
              </div>
            </div>
            <div className="right-side">
              <div>
                <Telemetry/>
              </div>
            </div>
          </div>
        </div>
        <Map/>
      </div>
    );
  }
}

export default Controls;