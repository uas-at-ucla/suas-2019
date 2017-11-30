import React, { Component } from 'react';
import './Telemetry.css';

var METERS_PER_SECOND_TO_MPH = 2.23694;

class Telemetry extends Component {

  render() {
    this.telemetry = this.props.appState.telemetry;
    this.telemetryText = {};
    if (this.telemetry) {
      this.telemetryText = {
        speed: this.round(this.telemetry["air_speed"] * METERS_PER_SECOND_TO_MPH, 1)
          + "mph",
        altitude: this.round(this.telemetry["gps_rel_alt"], 1) + " meters",
        position: this.round(this.telemetry["gps_lat"], 7) + ", "
          + this.round(this.telemetry["gps_lng"], 7),
        satellites: this.round(this.telemetry["gps_satellites"], 7),
        heading: this.round(this.telemetry["heading"], 7)
      }
    }

    return (
      <div className="Telemetry card text-white">
        <div id="full_state">
          <span id="armed_indicator">{this.props.appState.droneArmedStatus}</span>
          <span id="state_indicator">{this.props.appState.droneState}</span>
        </div>
        <table>
          <tbody>
            <tr id="telemetry_important">
              <td>Speed</td>
              <td id="telemetry_speed">{this.telemetryText.speed}</td>
            </tr>
            <tr>
              <td>Position</td>
              <td id="telemetry_position">{this.telemetryText.position}</td>
            </tr>
            <tr>
              <td>Heading</td>
              <td id="telemetry_heading">{this.telemetryText.heading}</td>
            </tr>
            <tr>
              <td>Altitude</td>
              <td id="telemetry_altitude">{this.telemetryText.altitude}</td>
            </tr>
            <tr>
              <td>Satellite Count</td>
              <td id="telemetry_satellites">{this.telemetryText.satellites}</td>
            </tr>
         </tbody>
        </table>
      </div>
    );
  }

  round(value, precision) {
    var multiplier = Math.pow(10, precision || 0);
    return Math.round(value * multiplier) / multiplier;
  }
}

export default Telemetry;