import React, { Component } from 'react';
import Graph from './Graph';
import './Analytics.css';

class Analytics extends Component {

  // Render charts base on checkboxes
  // The only props we want to pass into Graph component are
  // X and Y data points - this makes the component reuseable
  renderGraphs() {
    return (
      <Graph/>
    );
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
            <td><input type="checkbox"/></td>
            <td><input type="checkbox"/></td>
            <td><input type="checkbox"/></td>
            <td><input type="checkbox"/></td>
          </tr>
        </table>

        {this.renderGraphs()}

      </div>
    );
  }
}

export default Analytics;