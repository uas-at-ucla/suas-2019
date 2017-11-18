import React, { Component } from "react";

class Screen extends Component {

  render() {
    return(
      <div className="Screen">
        <div id="map"></div>
	    <div id="map_buttons">
	      <button id="show_sidebar_btn" className="btn btn-dark hidden">
	        <i className="fa fa-bars" aria-hidden="true"></i>
	      </button>
	      <button id="follow_drone_btn" className="btn btn-dark">
	        <i className="fa fa-location-arrow" aria-hidden="true"></i>
	      </button>
	    </div>
	    <div id="telemetry" className="card text-white">
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
    );
  }
}

export default Screen;