import React, { Component } from 'react';
import { Marker } from 'react-google-maps';

import GoogleMap from '../Utils/GoogleMap/GoogleMap';

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
        >
          <Marker position={{ lat: -34.397, lng: 150.644 }} />
        </GoogleMap>
      </div>
    );
  }
}

export default Map;
