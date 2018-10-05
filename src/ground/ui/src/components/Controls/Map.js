import React, { Component } from 'react';
import { Marker } from 'react-google-maps';

import GoogleMap from '../Utils/GoogleMap/GoogleMap';

class Map extends Component {
  render() {
    return (
      <div className="Map">
        <GoogleMap
          defaultZoom={8}
          defaultCenter={{ lat: -34.397, lng: 150.644 }}
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
