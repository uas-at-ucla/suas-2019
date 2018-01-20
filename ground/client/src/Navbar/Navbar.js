import React, { Component } from 'react';
import './Navbar.css'

import Option from './Option';
import Graph from '../Analytics/Graph';

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

  constructor(props) {
    super(props);
    this.state = {display_bat_graph: "block"};
    this.toggleBatteryGraph = this.toggleBatteryGraph.bind(this);
  }

  componentDidMount() {
    this.mountTime = Date.now();
  }

  toggleBatteryGraph() {
    if (this.state.display_bat_graph === "block")
      this.state.display_bat_graph = "none";
    else
      this.state.display_bat_graph = "block";
  }

  render() {
    var batteryAndFlightTime = null
    var batteryGraph = null
    if (this.props.appState.optionSelected === 'Home') {
      var flight_time = null;
      var battery_voltage = null;
      if (this.props.appState.telemetry) {
        flight_time = this.props.appState.telemetry.flight_time;
        flight_time = new Date(flight_time * 1000).toISOString().substr(11, 8);
        battery_voltage = this.props.appState.telemetry.voltage + 'v';
        batteryGraph = ( <div id="battery_graph_overlay" style={{display: this.state.display_bat_graph}}>
          <Graph dataName={"Battery Voltage"} color={"#e2c63b"}
          dataPoint={{x: (Date.now() - this.mountTime)/1000, y: this.props.appState.telemetry.voltage}}/>
          </div>
        );

      }
      batteryAndFlightTime = (
        <div id="flight_time_and_battery">
          <div id="flight_time">
            <div id="flight_time_value">{flight_time}</div>
            <div id="flight_time_label">Flight Time</div>
          </div>
          <div id="battery_v" onClick={this.toggleBatteryGraph}>
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
          {batteryGraph}
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
