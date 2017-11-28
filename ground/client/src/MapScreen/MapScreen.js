import React, { Component } from 'react';
import './MapScreen.css'
import Map from '../Map/Map'

class MapScreen extends Component {

  render = () => {
    return (
<div className="MapScreen">
  <Map ref="map"/>
  <button id="follow_drone_btn"
          className="btn btn-dark"
          onClick={() => this.refs.map.followDrone()}>
    <i className="fa fa-location-arrow" aria-hidden="true"></i>
  </button>
  <div id="map_buttons">
    <button id="show_sidebar_btn"
            className={`btn btn-dark ${this.props.isSidebarShown ? 'hidden' : null}`}
            onClick={this.props.showSidebar}>
      <i className="fa fa-bars" aria-hidden="true"></i>
    </button>
  </div>
  <div id="telemetry">
    <div id="full_state">
      <span id="armed_indicator">Offline</span>
      <span id="state_indicator"></span>
    </div>
    <table>
      <tbody>
        <tr id="telemetry_important">
          <td>Speed</td>
          <td id="telemetry_speed"></td>
        </tr>
        <tr>
          <td>Position</td>
          <td id="telemetry_position"></td>
        </tr>
        <tr>
          <td>Heading</td>
          <td id="telemetry_heading"></td>
        </tr>
        <tr>
          <td>Altitude</td>
          <td id="telemetry_altitude"></td>
        </tr>
        <tr>
          <td>Satellite Count</td>
          <td id="telemetry_satellites"></td>
        </tr>
     </tbody>
    </table>
  </div>
</div>
    )
  }
};

export default MapScreen
