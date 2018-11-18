import React, { Component } from 'react';
import { connect } from 'react-redux';

import './Telemetry.css';
import AttitudeIndicator from './AttitudeIndicator';
import Altimeter from './Altimeter';
import { selector } from '../../../store';

const mapStateToProps = state => {
  let derivedData = selector(state);
  console.log(derivedData);
  return {
    telemetry: state.telemetry,
    derivedData: derivedData
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
