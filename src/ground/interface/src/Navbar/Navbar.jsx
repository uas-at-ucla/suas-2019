import React, { Component } from "react";
import "./Navbar.css";

import Option from "./Option";
import Graph from "../Analytics/Graph";

class Navbar extends Component {
  // Display tab titles here
  getOptions() {
    return [
      { _id: 1, section: "Control" },
      { _id: 2, section: "Analytics" },
      { _id: 3, section: "Images" },
      { _id: 4, section: "Settings" },
      { _id: 5, section: "Training"},
    ];
  }

  renderOptions() {
    return this.getOptions().map(option => (
      <Option
        key={option._id}
        option={option}
        appState={this.props.appState}
        setAppState={this.props.setAppState}
      />
    ));
  }

  constructor(props) {
    super(props);
    this.state = { display_bat_graph: "block" };
    this.toggleBatteryGraph = this.toggleBatteryGraph.bind(this);
  }

  componentDidMount() {
    this.mountTime = Date.now();
  }

  toggleBatteryGraph() {
    if (this.state.display_bat_graph === "block")
      this.setState({display_bat_graph: "none"});
    else this.setState({display_bat_graph: "block"});
  }

  render() {
    var batteryAndFlightTime = null;
    var batteryGraph = null;
    if (this.props.appState.optionSelected === "Control") {
      var flight_time = null;
      var battery_voltage = null;
      if (this.props.appState.telemetry) {
        flight_time = this.props.appState.telemetry.status.flight_time;
        if (flight_time) flight_time = new Date(flight_time).toISOString().substr(11, 8);

        battery_voltage = this.props.appState.telemetry.sensors.battery_voltage;
        battery_voltage = this.round(battery_voltage, 2) + " volts";
        let cells = this.getBatteryCells(this.props.appState.telemetry.sensors.battery_voltage);

        let runtime = (Date.now() - this.mountTime) / 1000;
        let graph_options = {
          scales: {
            xAxes: [{
              display: false,
              ticks: {
                max: runtime,
                min: runtime - 10 * 3,
              }
            }],
            yAxes: [{
              display: true,
              position: 'right',
              ticks: {
                fontSize: 10,
                max: cells * 4.4,
                min: cells * 3.4,
              }
            }]
          },
          legend: {
            display: false
          },
          animation: {
            duration: 0
          },
          xAxes: [{
            override: {
              steps: 10,
            }
          }],
          yAxes: [{
          }],
          tooltips: {enabled: false},
          hover: {mode: null},
        };

        batteryGraph = (
          <div
            id="battery_graph_overlay"
            style={{ display: this.state.display_bat_graph }}
          >
            <Graph
              dataName={"Battery Voltage"}
              color={"#e2c63b"}
              dataPoint={{
                x: runtime,
                y: this.props.appState.telemetry.sensors.battery_voltage
              }}
              options={graph_options}
            />
          </div>
        );
      }
      batteryAndFlightTime = (
        <div id="flight_time_battery_and_graph">
          <div id="batteryGraph">
            {batteryGraph}
          </div>
          <div id="flight_time_and_battery">
            <div id="flight_time">
              <p>{flight_time}</p>
            </div>
            <div id="battery_v" onClick={this.toggleBatteryGraph}>
              <p>{battery_voltage}</p>
            </div>
          </div>
        </div>
      );
    }

    return (
      <div className="Navbar">
        <div className="nav">
          {this.renderOptions()}
        </div>
        {batteryAndFlightTime}
      </div>
    );
  }

  round(value, precision) {
    var multiplier = Math.pow(10, precision || 0);
    return Math.round(value * multiplier) / multiplier;
  }

  getBatteryCells = (voltage) => {
    return Math.floor(voltage / 3.8);
  }
}

export default Navbar;
