import React, { Component } from 'react';
import { connect } from 'react-redux';

import './Telemetry.css';
import AttitudeIndicator from './AttitudeIndicator';
import Altimeter from './Altimeter';

const mapStateToProps = state => {
  return {
    telemetry: state.telemetry
  };
};

class Telemetry extends Component {
  constructor(props) {
    super(props);

    this.update(JSON.parse(JSON.stringify(this.props.telemetry)));
  }


  update(rawTelmet) {
    if (rawTelmet != null) {
      this.telmet = {
        navball: {
          navX: rawTelmet["telemetry"]["sensors"]["gyroX"],
          navY: rawTelmet["telemetry"]["sensors"]["gyroY"],
          navZ: rawTelmet["telemetry"]["sensors"]["gyroZ"],
        },
        speed: 0,
        lat: rawTelmet["telemetry"]["sensors"]["latitude"],
        long: rawTelmet["telemetry"]["sensors"]["longitude"],
        heading: 0,
        alt: rawTelmet["telemetry"]["sensors"]["relativeAltitude"],
        satCount: rawTelmet["telemetry"]["sensors"]["gpsSatelliteCount"],
        gpsHdop: 0,
        gpsVdop: 0,
      }
    }
    else {
      this.telmet = {
        navball: {
          navX: 0,
          navY: 0,
          navZ: 0,
        },
        speed: 0,
        lat: 0,
        long: 0,
        heading: 0,
        alt: 0,
        satCount: 0,
        gpsHdop: 0,
        gpsVdop: 0,
      }
    }
  }


  render() {
    let telmet = JSON.parse(JSON.stringify(this.props.telemetry));

    this.update(telmet);

    console.log(telmet);

    return (
      <span className="Telemetry">
        <AttitudeIndicator data={this.telmet.navball} />
        
        <Altimeter/>
      </span>
    );
  }
}

export default connect(mapStateToProps)(Telemetry);
