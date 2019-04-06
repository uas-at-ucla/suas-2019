import React, { Component } from 'react';
import { connect } from 'react-redux';

import './Analytics.css';
import downloadToBrowser from 'utils/downloadToBrowser';

const mapStateToProps = state => {
  return {
    telemetry: state.telemetry.data,
    playback: state.telemetry.playback,
  };
};

const mapDispatchToProps = {
  togglePlayback: function() {
    return {type: 'TOGGLE_PLAYBACK'}
  }
}; //TODO Make action for recording telemetry, and action for starting playback

class Analytics extends Component {
  render() {
    return (
      <div className="Analytics">
        Telemetry: {JSON.stringify(this.props.telemetry)}
        <div>
          <button>
            Record!!! / Stop Recording (and save to file)!!
          </button>
          <button>
            Load Telemetry File / Unload File
          </button>
          <button>
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
    downloadToBrowser("telemetry.json", "file text content");
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(Analytics);