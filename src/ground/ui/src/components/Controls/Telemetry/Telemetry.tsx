import React, { Component } from "react";
import { connect } from "react-redux";

import "./Telemetry.css";
import AttitudeIndicator from "./AttitudeIndicator/AttitudeIndicator";
import Altimeter from "./Altimeter/Altimeter";
import Readout from "./Readout";
import { AppState } from "redux/store";

const FEET_PER_METER = 3.28084;
const KNOTS_PER_METER_SECOND = 1.94384;

const mapStateToProps = (state: AppState) => {
  return {
    telemetry: state.telemetry
  };
};

type Props = ReturnType<typeof mapStateToProps>;

const blankTelmet = {
  pingDelay: NaN,
  autopilotState: "",
  roll: 0,
  pitch: 0,
  yaw: Math.PI / 2,
  speed: 0,
  gpsFix: false,
  lat: 0,
  long: 0,
  heading: 0,
  alt: 0,
  satCount: 0,
  gpsHdop: 0,
  gpsVdop: 0
};

class Telemetry extends Component<Props> {
  private telmet = blankTelmet;

  private update() {
    let rawTelmet = this.props.telemetry.droneTelemetry;
    let pingDelay = this.props.telemetry.pingDelay;
    if (rawTelmet != null) {
      this.telmet = {
        pingDelay: pingDelay !== undefined ? pingDelay : NaN,
        autopilotState: rawTelmet["sensors"]["armed"]
          ? rawTelmet["sensors"]["autopilot_state"]
          : "Disarmed",
        roll: rawTelmet["sensors"]["roll"],
        pitch: rawTelmet["sensors"]["pitch"],
        yaw: rawTelmet["sensors"]["yaw"],
        speed:
          rawTelmet["sensors"]["gps_ground_speed"] * KNOTS_PER_METER_SECOND,
        gpsFix: rawTelmet["sensors"]["gps_fix"],
        lat: rawTelmet["sensors"]["latitude"],
        long: rawTelmet["sensors"]["longitude"],
        heading: rawTelmet["sensors"]["heading"],
        alt: rawTelmet["sensors"]["altitude"] * FEET_PER_METER,
        satCount: rawTelmet["sensors"]["gps_satellite_count"],
        gpsHdop: 0,
        gpsVdop: 0
      };
    } else {
      this.telmet = blankTelmet;
    }
    return this.telmet;
  }

  private readoutData() {
    return [
      {
        key: "Ping",
        values:
          this.telmet.pingDelay != NaN
            ? [this.telmet.pingDelay, " ms"]
            : ["Not Connected"]
      },
      {
        key: "State",
        values: [this.telmet.autopilotState]
      },
      {
        key: "Ground Speed",
        values: [this.telmet.speed.toFixed(3), " knots"]
      },
      {
        key: "Altitude MSL",
        values: [this.telmet.alt.toFixed(3), " feet"]
      },
      {
        key: "GPS Fix",
        values: [this.telmet.gpsFix ? "Yes" : "NO GPS FIX"]
      },
      {
        key: "Satellite Count",
        values: [this.telmet.satCount]
      }
    ];
  }

  public render() {
    this.update();

    // console.log(telmet);
    // console.log(this.readoutData());

    return (
      <span className="Telemetry">
        <Readout data={this.readoutData()} />
        <AttitudeIndicator data={this.telmet} />
        <Altimeter />
      </span>
    );
  }
}

export default connect(mapStateToProps)(Telemetry);
