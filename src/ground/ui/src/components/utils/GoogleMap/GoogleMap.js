import React, { Component } from 'react';
import { withScriptjs, withGoogleMap, GoogleMap } from 'react-google-maps';

import './GoogleMap.css';
import downloadToBrowser from 'utils/downloadToBrowser';

// UNCOMMENT TO USE MAPS OFFLINE (1/2)
import 'google_maps_js_api';
var google;

function getCustomTilesMapType() {
  return new google.maps.ImageMapType({
    getTileUrl: (coord, zoom) => {
      if (coord.y >= 0) {
        let url = null;
        try {
          // UNCOMMENT TO USE MAPS OFFLINE (2/2)
          url = require(`google_maps_js_api/map_images/${zoom}/mag-${zoom}_x-${coord.x}_y-${coord.y}.jpg`);
          if (!url) {
            throw new Error();
          }
        } catch(e) {
          // Note: 'v=844' may need to be updated to a new number from time to time.
          url = `https://khms0.googleapis.com/kh?v=844&hl=en-US&x=${coord.x}&y=${coord.y}&z=${zoom}`;
        }
  
        // UNCOMMENT TO TEST:
        // tileLoaded(coord, zoom);
  
        return url;
      }
    },
    tileSize: new google.maps.Size(256, 256),
    maxZoom: 20
  });
}

class GoogleMapWrapperComponent extends Component {
  setMapType = (mapComponent) => {
    if (mapComponent) {
      google = window.google;
      let customTilesMapType = getCustomTilesMapType();
      let map = mapComponent.context.__SECRET_MAP_DO_NOT_USE_OR_YOU_WILL_BE_FIRED; // lol don't worry
      map.mapTypes.set('customTiles', customTilesMapType);
      // map.setMapTypeId('customTiles');

      // pan smoothly when map center changes
      this.componentDidUpdate = (prevProps) => {
        if (prevProps.center !== this.props.center) {
          mapComponent.panTo(this.props.center);
        }
      }
  
      // UNCOMMENT TO TEST:
      // downloadTileListOnClick(map);
    }
  }
  
  GoogleMapComponent = withScriptjs(withGoogleMap((props) => (
    <GoogleMap {...props} ref={this.setMapType} />
  )));

  render() {
    return <this.GoogleMapComponent
      loadingElement={<div style={{ height: `100%` }} />}
      containerElement={<div style={{ height: `100%` }} />}
      mapElement={<div style={{ height: `100%` }} />}
      googleMapURL="https://maps.googleapis.com/maps/api/js?v=3.exp&libraries=geometry,drawing,visualization&key=AIzaSyBI-Gz_lh3-rKXFwlpElD7pInA60U-iK0c"
      {...this.props}
    />
  }
}

export default GoogleMapWrapperComponent;


// For testing only:
const tileBounds = {};
function tileLoaded(coord, zoom) {
  if (!tileBounds[zoom]) {
    tileBounds[zoom] = {
      left: Infinity, right: -Infinity, top: Infinity, bottom: -Infinity
    };
  }
  if (coord.x < tileBounds[zoom].left) {
    tileBounds[zoom].left = coord.x;
  }
  if (coord.x > tileBounds[zoom].right) {
    tileBounds[zoom].right = coord.x;
  }
  if (coord.y < tileBounds[zoom].top) {
    tileBounds[zoom].top = coord.y;
  }
  if (coord.y > tileBounds[zoom].bottom) {
    tileBounds[zoom].bottom = coord.y;
  }
}
function downloadTileListOnClick(map) {
  map.addListener('click', () => {
    let tileUrls = {};
    for (let zoom in tileBounds) {
      tileUrls[zoom] = [];
      for (let x = tileBounds[zoom].left; x <= tileBounds[zoom].right; x++) {
        for (let y = tileBounds[zoom].top; y <= tileBounds[zoom].bottom; y++) {
          tileUrls[zoom].push({
            x: x,
            y: y,
            url: `https://khms0.googleapis.com/kh?v=821&hl=en-US&x=${x}&y=${y}&z=${zoom}`
          });
        }
      }
    }
    downloadToBrowser("tileUrls.json", JSON.stringify(tileUrls));
  });
}
