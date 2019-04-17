import React, { Component } from 'react';
import { connect } from 'react-redux';

import './Analytics.css';
import downloadToBrowser from 'utils/downloadToBrowser';

const mapStateToProps = state => {
  return {
    telemetry: state.telemetry.data,
    playback: state.telemetry.playback,
  };
}

var loadedTelemetry;
var telemetryData = [];
var recording = false;
var usingLoaded = false;

const mapDispatchToProps = {
  togglePlayback: function() {
    return {type: 'TOGGLE_PLAYBACK'}
  }
}; //TODO Make action for recording telemetry, and action for starting playback

class Analytics extends Component {
  constructor(props) {
    super(props);
    this.handleSubmit = this.handleSubmit.bind(this);
    this.fileInput = React.createRef();
  }

  render() {
    if (recording){
      telemetryData.push(this.props.telemetry)
    }
    return (
      <div className="Analytics">

        Telemetry: {this.stringifyTelemetry}
        
        <div>
          <button onClick={this.toggleRecord}>
            Record!!! / Stop Recording (and save to file)!!
          </button>
        </div>

        <div>
        <input type="file" ref={this.fileInput} />
        </div>

        <div>
          <button onClick={this.handleSubmit}>
            Load Telemetry File / Unload File
          </button>
          
          <button onClick={this.runLoaded}>
            Run Loaded Telemetry / Pause Telemetry
          </button>
        </div>

        <div>
          <button>
            Save Current Interop Mission
          </button>
          <button>
            Load Interop Mission / Unload File
          </button>
        </div>
      </div>
    );
  }

  saveInteropMission() {
    //TODO: add the interop data to mapStateToProps (see Map.js)
    //Save it to a file using JSON.stringify and downloadToBrowser
  }

  loadInteropMission() {
    let data;
    //TODO load JSON data from a file
    this.props.dispatch({ type: 'INTEROP_DATA', payload: data });
  }

  downloadTelemetry() {
    downloadToBrowser("telemetry.json", JSON.stringify(telemetryData,null,2));
  }

  toggleRecord = () => {
    recording = !(recording);
    if (!recording){
      this.downloadTelemetry();
    }
  }

  handleSubmit(event) {
    event.preventDefault();
    alert(
      `Selected file - ${
        this.fileInput.current.files[0].name
      }`
    );
  }

  runLoaded = () => {
    var loaded = JSON.stringify(this.fileInput.current.files[0]);

    loadedTelemetry = JSON.parse(loaded);

    usingLoaded = true
  }

  stringifyTelemetry = string => {
    if (!(this.usingLoaded)) {
      return JSON.stringify(this.props.telemetry)
    }else{
      return JSON.stringify(this.loadedTelemetry)
    }
  }

}

export default connect(mapStateToProps, mapDispatchToProps)(Analytics);