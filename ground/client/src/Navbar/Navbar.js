import React, { Component } from 'react';
import './Navbar.css'

import Option from './Option';

class Navbar extends Component {

  // Display tab titles here
  getOptions() {
    return [
      { _id: 1, section: 'Home' },
      { _id: 2, section: 'Analytics' },
      { _id: 3, section: 'Images' },
    ];
  }

  renderOptions() {
    return this.getOptions().map((option) => (
      <Option key={option._id} option={option}
              appState={this.props.appState} setAppState={this.props.setAppState}/>
    ));
  }

  render() {
    var batteryAndFlightTime = null
    if (this.props.appState.optionSelected === 'Home') {
      var flight_time = null;
      var battery_voltage = null;
      if (this.props.appState.telemetry) {
        flight_time = this.props.appState.telemetry.flight_time;
        flight_time = new Date(flight_time * 1000).toISOString().substr(11, 8);
        battery_voltage = this.props.appState.telemetry.voltage + 'v';
      }
      batteryAndFlightTime = (
        <div id="flight_time_and_battery">
          <div id="flight_time">
            <div id="flight_time_value">{flight_time}</div>
            <div id="flight_time_label">Flight Time</div>
          </div>
          <div id="battery_v">
            <div id="battery_value">{battery_voltage}</div>
            <div id="battery_label">Battery</div>
          </div>
        </div>
      );
    }

    return (
      <div className="Navbar">
          <div className="nav">
            {this.renderOptions()}
            <button className={`btn btn-sm align-middle btn-outline-light ${!this.props.appState.interopBtnEnabled ? 'disabled' : null}`}
                    id="interop_btn" onClick={this.connectToInterop}>{this.props.appState.interopBtnText}</button>
          </div>
          {batteryAndFlightTime}
      </div>
    );
  }

  connectToInterop = () => {
    this.props.setAppState({interopBtnText: "Connecting..."});
    this.props.setAppState({interopBtnEnabled: false});
    this.props.socketEmit('connect_to_interop');
  }
}

export default Navbar;
