import React, { Component } from "react";
import { connect } from "react-redux";

import { selector, AppState } from "redux/store";
import "./Altimeter.css";

const FEET_PER_METER = 3.28084;
const MAX_ALT = 300; //feet

const mapStateToProps = (state: AppState) => {
  return {
    telemetry: state.telemetry.droneTelemetry,
    interopData: state.mission.interopData,
    mainFlyZone: selector(state).mission.mainFlyZone
  };
};

type Props = ReturnType<typeof mapStateToProps>;

class Altimeter extends Component<Props> {
  public render() {
    let relAltitude = 0;
    if (this.props.telemetry) {
      relAltitude =
        this.props.telemetry.sensors.relative_altitude * FEET_PER_METER;
    }
    let percentage = (relAltitude / MAX_ALT) * 100;

    let min = 0;
    let max = Infinity;
    if (this.props.telemetry && this.props.interopData) {
      let homeAlt = this.props.telemetry.sensors.home_altitude * FEET_PER_METER;
      min = this.props.mainFlyZone.altitudeMin - homeAlt;
      max = this.props.mainFlyZone.altitudeMax - homeAlt;
    }
    if (max > MAX_ALT) max = MAX_ALT;

    return (
      <div className="Altimeter">
        <div className="progress-bars">
          <div className="progress-bar">
            <div
              className={`filler ${
                relAltitude < min
                  ? "too-low"
                  : relAltitude > max
                  ? "too-high"
                  : "default"
              }`}
              style={{ height: `${percentage > 0 ? percentage : 0}%` }}
            >
              {relAltitude.toFixed(0)} ft rel
            </div>
          </div>
          <div className="progress-bar">
            <div
              className="filler-min"
              style={{ height: `${(min / MAX_ALT) * 100}%` }}
            ></div>
          </div>
          <div className="progress-bar">
            <div
              className="filler-max"
              style={{ height: `${(max / MAX_ALT) * 100}%` }}
            ></div>
          </div>
        </div>
      </div>
    );
  }
}

export default connect(mapStateToProps)(Altimeter);
