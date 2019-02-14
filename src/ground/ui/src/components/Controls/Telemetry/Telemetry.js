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
  constructor(props) {
    super(props);

    this.state = {
      navX: 0,
      navY: 0,
      navZ: 0,
    };
  }


  render() {
    let telmet = JSON.parse(JSON.stringify(this.props.telemetry));

    console.log(telmet);
    if(telmet != null) {
      console.log(telmet["telemetry"]["sensors"]["gyroX"]);
      console.log(telmet["telemetry"]["sensors"]["gyroY"]);
      console.log(telmet["telemetry"]["sensors"]["gyroZ"]);
      console.log();
    }
    

    return (
      <span className="Telemetry">
        <AttitudeIndicator 
          x={telmet == null ? 0 : telmet["telemetry"]["sensors"]["gyroX"]}
          y={telmet == null ? 0 : telmet["telemetry"]["sensors"]["gyroY"]} 
          z={telmet == null ? 0 : telmet["telemetry"]["sensors"]["gyroZ"]}/>
        
        <Altimeter/>
      </span>
    );
  }
}

export default connect(mapStateToProps)(Telemetry);
