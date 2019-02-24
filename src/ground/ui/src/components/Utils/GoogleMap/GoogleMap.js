import React from 'react';
import { withGoogleMap, GoogleMap } from 'react-google-maps';

import './GoogleMap.css'

const GoogleMapComponent = withGoogleMap((props) => (
  <GoogleMap {...props} />
));

const GoogleMapWrapperComponent = (props) => (
  <GoogleMapComponent
    loadingElement={<div style={{ height: `100%` }} />}
    containerElement={<div style={{ height: `100%` }} />}
    mapElement={<div style={{ height: `100%` }} />}
    {...props}
  />
);

export default GoogleMapWrapperComponent;