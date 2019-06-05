import React, { Component } from 'react';
import { connect } from 'react-redux';

import { selector } from 'redux/store';
import './Altimeter.css';

const FEET_PER_METER = 3.28084;
const MAX_ALT = 300; //feet

const mapStateToProps = state => {
  return {
    telemetry: state.telemetry.droneTelemetry,
    interopData: state.mission.interopData,
    mainFlyZone: selector(state).mission.mainFlyZone
  };
};
class Altimeter extends Component {
  render() {
    let rel_altitude = 0;
    if (this.props.telemetry) {
      rel_altitude = this.props.telemetry.sensors.relative_altitude * FEET_PER_METER;
    }
    let percentage = (rel_altitude / MAX_ALT) * 100;

    let min = 0;
    let max = Infinity;
    if (this.props.telemetry && this.props.interopData) {
      let home_alt = this.props.telemetry.sensors.home_altitude * FEET_PER_METER;
      min = this.props.mainFlyZone.altitudeMin - home_alt;
      max = this.props.mainFlyZone.altitudeMax - home_alt;
    }
    if (max > MAX_ALT) max = MAX_ALT;   

    return (
      <div className="Altimeter"> 
        <div className="progress-bars">
          <div className="progress-bar">
            <div className={`filler ${rel_altitude < min ? "too-low" : rel_altitude > max ? "too-high" : "default"}`} style={{ height: `${percentage > 0 ? percentage : 0}%` }}>
              {rel_altitude.toFixed(0)} ft rel
            </div>
          </div>
          <div className="progress-bar">
            <div className="filler-min" style={{ height: `${(min/MAX_ALT)*100}%` }}></div>
          </div>
          <div className="progress-bar">
            <div className="filler-max" style={{ height: `${(max/MAX_ALT)*100}%` }}></div>
          </div>
        </div>
      </div>
    );
  }  
}

export default connect(mapStateToProps)(Altimeter);