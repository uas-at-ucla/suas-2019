import React, { Component } from 'react';
import { connect } from 'react-redux';

import './Telemetry.css';
import AttitudeIndicator from './AttitudeIndicator';
import Altimeter from './Altimeter';

const mapStateToProps = state => {
  return {
    telemetry: state.telemetry
  };
};

class Telemetry extends Component {
  render() {
    return (
      <span className="Telemetry">
        <div className="Readout">{JSON.stringify(this.props.telemetry)}</div>
        <AttitudeIndicator/>
        <Altimeter/>
      </span>
    );
  }
}

export default connect(mapStateToProps)(Telemetry);
