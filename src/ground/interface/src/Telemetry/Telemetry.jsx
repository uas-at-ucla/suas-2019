import React, { Component } from "react";
import "./Telemetry.css";

const METERS_PER_SECOND_TO_MPH = 2.23694;
const METERS_PER_FOOT = 0.3048;
const MAX_ALTITUDE = 400;

class Telemetry extends Component {
  render() {
    this.telemetry = this.props.appState.telemetry;
    this.telemetryText = {};
    var filteredAltitude = 0;

    if (this.telemetry) {
      this.telemetryText = {
        speed_metric:
          this.round(
            this.telemetry.sensors["gps_ground_speed"],
            1
          ) + " m/s",
        speed_imperial:
          this.round(
            this.telemetry.sensors["gps_ground_speed"] *
              METERS_PER_SECOND_TO_MPH,
            1
          ) + " mph",
        altitude_metric:
          this.round(this.telemetry.sensors["relative_altitude"], 1) +
          " meters",
        altitude_imperial:
          this.round(this.telemetry.sensors["relative_altitude"]
           / METERS_PER_FOOT, 1) +
          " feet",
        position:
          this.round(this.telemetry.sensors["latitude"], 7) +
          ", " +
          this.round(this.telemetry.sensors["longitude"], 7),
        satellites: this.round(
          this.telemetry.sensors["gps_satellite_count"],
          7
        ),
        heading: this.round(this.telemetry.sensors["heading"], 7),
        gps_eph: this.round(this.telemetry.sensors["gps_eph"], 7),
        gps_epv: this.round(this.telemetry.sensors["gps_epv"], 7)
      };

      filteredAltitude = this.round(
        this.telemetry.sensors["relative_altitude"],
        1
      );
    }

    var altimeterHeight = filteredAltitude / MAX_ALTITUDE * 90;
    var altimeterBottom = 10;
    var altimeterColor = "#007BFF";

    if (altimeterHeight < 0) {
      altimeterHeight *= -1;
      altimeterBottom = 10 - altimeterHeight;
      altimeterColor = "#FF7B00";
    }

    var boundaryAltLine = null;
    if (
      this.telemetry &&
      this.props.homeState.mission &&
      this.props.homeState.mission.fly_zones &&
      this.props.homeState.mission.fly_zones[0]
    ) {
      var groundAlt =
        this.telemetry.sensors["altitude"] -
        this.telemetry.sensors["relative_altitude"];
      var boundaryAlt =
        this.props.homeState.mission.fly_zones[0].altitude_msl_max *
          METERS_PER_FOOT -
        groundAlt;
      boundaryAltLine = (
        <p
          id="altimeterBoundaryAlt"
          style={{
            textAlign: "center",
            position: "absolute",
            bottom: 10 + boundaryAlt / MAX_ALTITUDE * 90 + "%",
            width: "100%",
            borderBottom: "1px solid #ff0000",
            zIndex: 100,
            margin: "auto"
          }}
        >
          <span className="metric">{this.round(boundaryAlt, 1)}m</span>
          <span className="imperial">{this.round(boundaryAlt/METERS_PER_FOOT, 0)}ft</span>
        </p>
      );
    }

    return (
      <div className="Telemetry">
        <div className="card text-white" id="telemetryNumbers">
          <div id="full_state">
            <span id="armed_indicator">{this.props.appState.droneState} </span>
            <span id="ping"
              style={{
                backgroundColor: this.pingColor()
              }}
            >
              {this.props.appState.drone_ping_ms === null ? "!!!" : Number(this.props.appState.drone_ping_ms).toFixed(1)+"ms"}
            </span>
          </div>
          <table>
            <col width="1%"/>
            <tbody>
              <tr id="telemetry_important">
                <td>Speed</td>
                <td id="telemetry_speed">
                  <span class="metric">{this.telemetryText.speed_metric}</span>
                  <span class="imperial">{this.telemetryText.speed_imperial}</span>
                </td>
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
                <td id="telemetry_altitude">
                  <span class="metric">{this.telemetryText.altitude_metric}</span>
                  <span class="imperial">{this.telemetryText.altitude_imperial}</span>
                </td>
              </tr>
              <tr>
                <td>Satellite Count</td>
                <td id="telemetry_satellites">
                  {this.telemetryText.satellites}
                </td>
              </tr>
              <tr>
                <td>GPS HDOP (Signal)</td>
                <td id="telemetry_gps_eph">{this.telemetryText.gps_eph}</td>
              </tr>
              <tr>
                <td>GPS VDOP (Signal)</td>
                <td id="telemetry_gps_epv">{this.telemetryText.gps_epv}</td>
              </tr>
            </tbody>
          </table>
        </div>

        <div className="card text-white" id="altimeter">
          <p id="altimeterMaxAlt">
            <span className="metric">{MAX_ALTITUDE}m</span>
            <span className="imperial">{this.round(MAX_ALTITUDE/METERS_PER_FOOT, 0)}ft</span>
          </p>
          {boundaryAltLine}
          <p id="altimeterMinAlt">
            <span className="metric">0m</span>
            <span className="imperial">0ft</span>
          </p>
          <div
            id="altimeterAltitudeIndicator"
            style={{
              height: altimeterHeight + "%",
              background: altimeterColor,
              bottom: altimeterBottom + "%"
            }}
          />
        </div>
      </div>
    );
  }

  round(value, precision) {
    var multiplier = Math.pow(10, precision || 0);
    return Math.round(value * multiplier) / multiplier;
  }

  pingColor() {
    let ms = this.props.appState.drone_ping_ms;
    if (ms === null) {
      return "rgba(255, 0, 0, 1)";
    } else {
      let max_ping = 100;
      let hue = (1 - Math.min(ms, max_ping)/max_ping) / 3;
      let rgb = this.HSVtoRGB(hue, 1, 1);
      return "rgba("+rgb.r+","+rgb.g+","+rgb.b+", 0.5)";
    }
  }

  // From https://stackoverflow.com/questions/17242144/javascript-convert-hsb-hsv-color-to-rgb-accurately
  HSVtoRGB(h, s, v) {
    var r, g, b, i, f, p, q, t;
    if (arguments.length === 1) {
        s = h.s, v = h.v, h = h.h;
    }
    i = Math.floor(h * 6);
    f = h * 6 - i;
    p = v * (1 - s);
    q = v * (1 - f * s);
    t = v * (1 - (1 - f) * s);
    switch (i % 6) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }
    return {
        r: Math.round(r * 255),
        g: Math.round(g * 255),
        b: Math.round(b * 255)
    };
  }
}

export default Telemetry;
