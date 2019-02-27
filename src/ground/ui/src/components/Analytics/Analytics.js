import React, { Component } from 'react';
import { connect } from 'react-redux';

import './Analytics.css';
import downloadToBrowser from '../../utils/downloadToBrowser';

const mapStateToProps = state => {
  return {
    telemetry: state.telemetry
  };
};

const mapDispatchToProps = {}; //TODO Make action for recording telemetry

class Analytics extends Component {
  render() {
    return (
      <div className="Analytics">
        Telemetry: {JSON.stringify(this.props.telemetry)}
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
    );
  }

  downloadTelemetry() {
    downloadToBrowser("telemetry.json", "test content");
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(Analytics);