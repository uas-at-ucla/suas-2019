import React, { Component } from 'react';
import Graph from './Graph';
import Checkbox from './Checkbox';
import './Analytics.css';

class Analytics extends Component {

  constructor(props) {
    super(props);
    this.state = {
      showGraphs: {
        velocity_x: "none",
        velocity_y: "none",
        velocity_z: "none",
        voltage: "none"
      }
    }
    this.updateGraphVis = this.updateGraphVis.bind(this);
  }

  updateGraphVis(graphName, visibility) {
    console.log(graphName + ": " + visibility);
    if (visibility) {
        this.setState((prevState) => {
        let newState = {showGraphs:{...prevState.showGraphs}};
        newState.showGraphs[graphName] = "block";
        return newState;
      });
    }
    else {
        this.setState((prevState) => {
        let newState = {showGraphs:{...prevState.showGraphs}};
        newState.showGraphs[graphName] = "none";
        return newState;
      });

    }
  }

  // Render charts base on checkboxes
  // The only props we want to pass into Graph component are
  // X and Y data points - this makes the component reuseable
  renderGraphs() {
    if (this.telemetry) {
      return (
        <div>
          <div style={{display:this.state.showGraphs["velocity_x"]}}>
            <Graph dataName="X Velocity" color="#f00"
              dataPoint={{x: this.telemetry["flight_time"], y: this.telemetry["velocity_x"]}} />
          </div>
          <div style={{display:this.state.showGraphs["velocity_y"]}}>
            <Graph dataName="Y Velocity" color="#0f0"
              dataPoint={{x: this.telemetry["flight_time"], y: this.telemetry["velocity_y"]}} />
          </div>
          <div style={{display:this.state.showGraphs["velocity_z"]}}>
            <Graph dataName="Z Velocity" color="#00f"
              dataPoint={{x: this.telemetry["flight_time"], y: this.telemetry["velocity_z"]}} />
          </div>
          <div style={{display:this.state.showGraphs["voltage"]}}>
            <Graph dataName="Battery Voltage" color="rgba(255,140,0,1)"
              dataPoint={{x: this.telemetry["flight_time"], y: this.telemetry["voltage"]}} />
          </div>
        </div>
      );
    }
  }

  render() {
  	this.telemetry = this.props.appState.telemetry;

    return (
      <div className="Analytics col-md-offset-1 col-md-10">
        <h1>Analytics</h1>

        <table class="text-center" width="80%">
          <tr>
            <th>X Velocity</th>
            <th>Y Velocity</th>
            <th>Z Velocity</th>
            <th>Voltage</th>
          </tr>
          <tr>
            <td><Checkbox valName="velocity_x" updateVal={this.updateGraphVis}/></td>
            <td><Checkbox valName="velocity_y" updateVal={this.updateGraphVis}/></td>
            <td><Checkbox valName="velocity_z" updateVal={this.updateGraphVis}/></td>
            <td><Checkbox valName="voltage" updateVal={this.updateGraphVis}/></td>
          </tr>
        </table>

        {this.renderGraphs()}

      </div>
    );
  }
}

export default Analytics;