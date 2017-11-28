import React, { Component } from 'react';
import './Dashboard.css'

class dashboard extends Component {
  render() {
    return (
<div className="Dashboard">
  <div id="sidebar_content">
    <div className="card">
      <div className="card-body">
        <h4 className="card-text">Mission 1</h4>
      </div>
    </div>
    <div className="card">
      <div className="card-body">
        <button className="btn btn-outline-primary">Add Mission Waypoints</button><br/><br/>
        <button className="btn btn-outline-primary">Add New Waypoint</button><br/><br/>
        <button className="btn btn-outline-success">Execute!</button>
      </div>
    </div>
    <div className="card">
      <h4 className="card-header">Received Images</h4>
    </div>
  </div>
</div>
    );
  }
}

export default dashboard;
