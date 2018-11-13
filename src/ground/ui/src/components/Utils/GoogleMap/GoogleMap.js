import React from 'react';
import { withScriptjs, withGoogleMap, GoogleMap } from 'react-google-maps';

import './GoogleMap.css'

const GoogleMapComponent = withScriptjs(withGoogleMap((props) => (
  <GoogleMap {...props} />
)));

const GoogleMapWrapperComponent = (props) => (
  <GoogleMapComponent
    googleMapURL="https://maps.googleapis.com/maps/api/js?v=3.exp&libraries=geometry,drawing,visualization&key=AIzaSyBI-Gz_lh3-rKXFwlpElD7pInA60U-iK0c"
    loadingElement={<div style={{ height: `100%`, minHeight: `500px` }} />}
    containerElement={<div style={{ height: `100%`, minHeight: `500px` }} />}
    mapElement={<div style={{ height: `100%`, minHeight: `500px` }} />}
    {...props}
  />
);

export default GoogleMapWrapperComponent;
