import React, { Component } from 'react';

class Analytics extends Component {

  render() {
  	this.telemetry = this.props.appState.telemetry;

    return (
      <div className="Analytics">
        <h1>Render analytics here</h1>
        <h2>Here's the telemetry object</h2>
        <span>{JSON.stringify(this.telemetry)}</span>
      </div>
    );
  }
}

export default Analytics;