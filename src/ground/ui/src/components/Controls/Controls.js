import React, { Component } from 'react';

import './Controls.css';
import Map from './Map';
import Telemetry from './Telemetry/Telemetry';
import DroneActions from './DroneActions';
import Navball from './Navball/Navball';

class Controls extends Component {
  render() {
    return (
      <div className="Controls">
        <div className="map-overlay">
          <div>
            <div className="left-side">
              <div>
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
        <Navball />
      </div>
    );
  }
}

export default Controls;