import React, { Component } from 'react';
import { Marker, Circle } from 'react-google-maps';
import { connect } from 'react-redux';

import rover from './icons/rover.png';

const mapStateToProps = state => {
  return {
    telemetry: state.telemetry.droneTelemetry,
    ugvStatus: state.telemetry.ugvStatus
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

        {this.props.ugvStatus && this.props.ugvStatus.location ?
          <Marker
            position={{lat: this.props.ugvStatus.location.latitude, lng: this.props.ugvStatus.location.longitude}}
            icon={{
              url: rover,
              scaledSize: {width: 25, height: 25},
              anchor: {x: 12.5, y: 12.5}
            }}
          />
        : null}
      </span>
    );
  }
}

export default connect(mapStateToProps)(DroneMarker);
