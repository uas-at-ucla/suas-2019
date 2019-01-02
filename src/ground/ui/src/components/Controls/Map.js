import React, { Component } from 'react';
import { Marker, InfoWindow } from 'react-google-maps';
import { connect } from 'react-redux';

import GoogleMap from '../Utils/GoogleMap/GoogleMap';
import missionActions from '../../actions/missionActions';
import { selector } from '../../store';

const mapStateToProps = state => {
  let derivedData = selector(state);
  return {
    commandPoints: derivedData.missionPlan.commandPoints,
    protoInfo: derivedData.missionPlan.protoInfo
  };
};

const mapDispatchToProps = missionActions;

class Map extends Component {
  render() {
    return (
      <div className="Map">
        <GoogleMap
          defaultZoom={16}
          defaultCenter={{ lat: 38.147483, lng: -76.427778 }}
          defaultMapTypeId="satellite"
          defaultOptions={{
            disableDefaultUI: true,
            disableDoubleClickZoom: true,
            scaleControl: true
          }}
          onDblClick={this.mapDblClick}
        >
          {this.props.commandPoints.map(commandPoint => 
            commandPoint ?
              <Marker {...commandPoint.marker} key={commandPoint.id}>
                <InfoWindow {...commandPoint.infobox}>
                  <div className="map-infobox">
                    {commandPoint.infobox.content}
                  </div>
                </InfoWindow>
              </Marker>
            : null
          )}
          <Marker position={{ lat: -34.397, lng: 150.644 }} />
        </GoogleMap>
      </div>
    );
  }

  mapDblClick = (event) => {
    this.addWaypointCommand(event.latLng.lat(), event.latLng.lng());
  }

  addWaypointCommand = (lat, lng) => {
    let defaultWaypointCommand = { goal: {
      latitude: lat,
      longitude: lng,
      altitude: 100
    }}
    this.props.addWaypointCommand(defaultWaypointCommand, this.props.protoInfo);
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(Map);
