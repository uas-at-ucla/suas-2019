import React, { Component } from 'react';
import { connect } from 'react-redux';

import './Telemetry.css';
import AttitudeIndicator from './AttitudeIndicator/AttitudeIndicator';
import Altimeter from './Altimeter/Altimeter';
import Readout from './Readout';

const mapStateToProps = state => {
  return {
    telemetry: state.telemetry,
  };
};

class Telemetry extends Component {
  update() {
    let rawTelmet = this.props.telemetry.droneTelemetry;
    let pingDelay = this.props.telemetry.pingDelay;
    if (rawTelmet != null) {
      this.telmet = {
        pingDelay: pingDelay,
        navX: rawTelmet["sensors"]["gyroX"],
        navY: rawTelmet["sensors"]["gyroY"],
        navZ: rawTelmet["sensors"]["gyroZ"],
        speed: 0,
        lat: rawTelmet["sensors"]["latitude"],
        long: rawTelmet["sensors"]["longitude"],
        heading: 0,
        alt: rawTelmet["sensors"]["relativeAltitude"],
        satCount: rawTelmet["sensors"]["gpsSatelliteCount"],
        gpsHdop: 0,
        gpsVdop: 0,
      }
    }
    else {
      this.telmet = {
        pingDelay: null,
        navX: 0,
        navY: 0,
        navZ: 0,
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

  readoutData() {
    return [
      {
        key: "Ping",
        values: [this.telmet.pingDelay, " ms"]
      },
      {
        key: "Speed",
        values: [this.telmet.speed.toFixed(3) , " mph"]
      },
      {
        key: "Position",
        values: [this.telmet.lat.toFixed(3) , ", " , this.telmet.long.toFixed(3)]
      },
      {
        key: "Altitude",
        values: [this.telmet.alt.toFixed(3) , " meters"]
      },
      {
        key: "Satellite Count",
        values: [this.telmet.satCount]
      },
    ];
  }

  render() {
    this.update();

    // console.log(telmet);
    // console.log(this.readoutData());

    return (
      <span className="Telemetry">
        <Readout data={this.readoutData()} />
        <AttitudeIndicator data={this.telmet} />
        <Altimeter/>
      </span>
    );
  }
}

export default connect(mapStateToProps)(Telemetry);
