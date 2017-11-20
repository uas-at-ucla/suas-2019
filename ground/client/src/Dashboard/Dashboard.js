import React from 'react';
import './Dashboard.css'

const dashboard = (props) => {
  return (
    <div className="Dashboard">
      <h3 id="sidebar_title">
        <p id="sidebar_title_text">Dashboard</p>
        <button id="hide_sidebar_btn" className="btn btn-light" onClick={props.close}>&#10005;</button>
      </h3>
      <div id="sidebar_content">
        <div className="card">
          <h4 className="card-header">Missions</h4>
          <div className="card-body">
            <p className="card-text">Mission 1</p>
          </div>
        </div>
        <div className="card">
          <h4 className="card-header">Commands</h4>
          <div className="card-body">
            <p className="card-text">List of Waypoints goes here</p>
            <button className="btn btn-outline-primary">Add Mission Waypoints</button>
            <button className="btn btn-outline-primary">Add New Waypoint</button>
            <button className="btn btn-outline-success">Execute!</button>
          </div>
        </div>
        <div className="card">
          <h4 className="card-header">Received Images</h4>
          <div className="card-body">
            <p className="card-text">Feature coming... eventually</p>
          </div>
        </div>
      </div>
    </div>
  )
};

export default dashboard