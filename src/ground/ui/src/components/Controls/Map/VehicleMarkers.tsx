import React, { Component } from "react";
import { Marker, Circle } from "react-google-maps";
import { connect } from "react-redux";

import google from "components/utils/GoogleMap/google";
import rover from "./icons/rover.png";
import { AppState } from "redux/store";

const mapStateToProps = (state: AppState) => {
  return {
    telemetry: state.telemetry.droneTelemetry,
    ugvStatus: state.telemetry.ugvStatus
  };
};

type Props = ReturnType<typeof mapStateToProps>;

class DroneMarker extends Component<Props> {
  public render() {
    var pos = this.props.telemetry
      ? {
          lat: this.props.telemetry.sensors.latitude,
          lng: this.props.telemetry.sensors.longitude
        }
      : undefined;
    return (
      <span>
        {this.props.telemetry ? (
          <span>
            <Circle
              center={pos}
              radius={this.props.telemetry.sensors.gps_eph}
              options={{ clickable: false }}
            />
            <Marker
              position={pos}
              icon={{
                path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
                strokeColor: "#FFFFFF",
                strokeOpacity: 0.8,
                strokeWeight: 3,
                fillColor: "#0000FF",
                fillOpacity: 0.5,
                scale: 7,
                rotation: this.props.telemetry.sensors.heading,
                anchor: new google.maps.Point(0, 2.5)
              }}
            ></Marker>
          </span>
        ) : null}

        {this.props.ugvStatus && this.props.ugvStatus.location ? (
          <Marker
            position={{
              lat: this.props.ugvStatus.location.latitude,
              lng: this.props.ugvStatus.location.longitude
            }}
            icon={{
              url: rover,
              scaledSize: new google.maps.Size(25, 25),
              anchor: new google.maps.Point(12.5, 12.5)
            }}
          />
        ) : null}
      </span>
    );
  }
}

export default connect(mapStateToProps)(DroneMarker);
