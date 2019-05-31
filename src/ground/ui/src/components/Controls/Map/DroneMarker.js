import React, { Component } from 'react';
import { Marker, Circle } from 'react-google-maps';
import { connect } from 'react-redux';

const mapStateToProps = state => {
  return {
    telemetry: state.telemetry.droneTelemetry,
  };
};

class DroneMarker extends Component {
  render() {
    if (this.props.telemetry) {
      var pos = {
        lat: this.props.telemetry.sensors.latitude,
        lng: this.props.telemetry.sensors.longitude
      }
    }
    return (
      <span>
        {this.props.telemetry ?
          <span>
            <Circle
              center={pos}
              radius={this.props.telemetry.sensors.gps_eph}
              options={{clickable: false}}
            />
            <Marker
              position={pos}
              draggable={true}
              icon={{
                path: window.google ? window.google.maps.SymbolPath.FORWARD_CLOSED_ARROW : null,
                strokeColor: '#FFFFFF',
                strokeOpacity: 0.8,
                strokeWeight: 3,
                fillColor: '#0000FF',
                fillOpacity: 0.5,
                scale: 7,
                rotation: this.props.telemetry.sensors.heading,
                anchor: {x: 0, y: 2.5}
              }}
            >
            </Marker>
          </span>
        : null}
      </span>
    );
  }
}

export default connect(mapStateToProps)(DroneMarker);
