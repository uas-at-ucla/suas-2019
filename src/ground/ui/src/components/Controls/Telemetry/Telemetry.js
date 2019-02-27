import React, { Component } from 'react';
import { connect } from 'react-redux';

import './Telemetry.css';
import AttitudeIndicator from './AttitudeIndicator';
import Altimeter from './Altimeter';
import Readout from './Readout';


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
        navX: rawTelmet["telemetry"]["sensors"]["gyroX"],
        navY: rawTelmet["telemetry"]["sensors"]["gyroY"],
        navZ: rawTelmet["telemetry"]["sensors"]["gyroZ"],
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
    let telmet = JSON.parse(JSON.stringify(this.props.telemetry));

    this.update(telmet);

    console.log(telmet);
    console.log(this.readoutData());

    return (
      <span className="Telemetry">
        <AttitudeIndicator data={this.telmet} />
        <Readout data={this.readoutData()} />
        <Altimeter/>
      </span>
    );
  }
}

export default connect(mapStateToProps)(Telemetry);
