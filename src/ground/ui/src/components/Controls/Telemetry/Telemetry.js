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
      <div className="Telemetry">
        <div>{this.props.telemetry}</div>
        <AttitudeIndicator/>
        <Altimeter/>
      </div>
    );
  }
}

export default connect(mapStateToProps)(Telemetry);
