import React, { Component } from 'react';
import { connect } from 'react-redux';

import './Controls.css';
import Map from './Map';
import Telemetry from './Telemetry/Telemetry';
import DroneActions from './DroneActions';
import MissionPlannerContainer from './MissionPlannerContainer/MissionPlannerContainer';
import Cosmetics from 'components/Utils/Cosmetics/Cosmetics';

class Controls extends Component {
  render() {
    return (
      <div className="Controls">
        <div className="map-overlay">
          <div>
            <span className="left-side">
              <span className="top-left">
                <Cosmetics onClick={this.centerOnDrone}/>
                <MissionPlannerContainer/>
              </span>
              <span className="bottom-left">
                <DroneActions/>
              </span>
            </span>
            <span className="right-side">
              <Telemetry/>
            </span>
          </div>
        </div>
        <Map/>
      </div>
    );
  }

  centerOnDrone = () => {
    this.props.dispatch({type: 'CENTER_ON_DRONE'});
  }
}

export default connect()(Controls);