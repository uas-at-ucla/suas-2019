import React, { Component } from 'react';
import { connect } from 'react-redux';

const mapStateToProps = state => {
  return {
    telemetry: state.get('telemetry')
  };
};

class Telemetry extends Component {
  render() {
    return (
      <div className="Telemetry">
        {this.props.telemetry}
      </div>
    );
  }
}

export default connect(mapStateToProps)(Telemetry);
