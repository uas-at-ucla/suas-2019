import React, { Component } from 'react';
import { connect } from 'react-redux';

import './Telemetry.css';
import AttitudeIndicator from './AttitudeIndicator/AttitudeIndicator';
import Altimeter from './Altimeter/Altimeter';
import Readout from './Readout';

const FEET_PER_METER = 3.28084;
const KNOTS_PER_METER_SECOND = 1.94384;

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
        autopilotState: rawTelmet["sensors"]["armed"] ? rawTelmet["sensors"]["autopilot_state"] : "Disarmed",
        roll: rawTelmet["sensors"]["roll"],
        pitch: rawTelmet["sensors"]["pitch"],
        yaw: rawTelmet["sensors"]["yaw"],
        speed: rawTelmet["sensors"]["gps_ground_speed"] * KNOTS_PER_METER_SECOND,
        gps_fix: rawTelmet["sensors"]["gps_fix"],
        lat: rawTelmet["sensors"]["latitude"],
        long: rawTelmet["sensors"]["longitude"],
        heading: rawTelmet["sensors"]["heading"],
        alt: rawTelmet["sensors"]["altitude"] * FEET_PER_METER,
        satCount: rawTelmet["sensors"]["gps_satellite_count"],
        gpsHdop: 0,
        gpsVdop: 0,
      }
    }
    else {
      this.telmet = {
        pingDelay: null,
        autopilotState: null,
        roll: 0,
        pitch: 0,
        yaw: Math.PI/2,
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
        values: this.telmet.pingDelay != null ? [this.telmet.pingDelay, " ms"] : ["Not Connected"]
      },
      {
        key: "State",
        values: [this.telmet.autopilotState]
      },
      {
        key: "Ground Speed",
        values: [this.telmet.speed.toFixed(3) , " knots"]
      },
      {
        key: "Altitude MSL",
        values: [this.telmet.alt.toFixed(3) , " feet"]
      },
      {
        key: "GPS Fix",
        values: [this.telmet.gps_fix ? "Yes" : "NO GPS FIX"]
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
