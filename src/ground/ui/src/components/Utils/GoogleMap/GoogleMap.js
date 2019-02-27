import React from 'react';
import { withGoogleMap, GoogleMap } from 'react-google-maps';

import './GoogleMap.css';
import downloadToBrowser from '../../../utils/downloadToBrowser';

const google = window.google;

const customTilesMapType = new google.maps.ImageMapType({
  getTileUrl: (coord, zoom) => {
    if (coord.y >= 0) {
      let url = null;
      try {
        url = require(`google_maps_js_api/map_images/${zoom}/mag-${zoom}_x-${coord.x}_y-${coord.y}.jpg`);
      } catch(e) {
        url = `https://khms0.googleapis.com/kh?v=821&hl=en-US&x=${coord.x}&y=${coord.y}&z=${zoom}`;
      }

      // UNCOMMENT TO TEST:
      // tileLoaded(coord, zoom);

      return url;
    }
  },
  tileSize: new google.maps.Size(256, 256),
  maxZoom: 20
});

function setMapType(mapComponent) {
  if (mapComponent) {
    let map = mapComponent.context.__SECRET_MAP_DO_NOT_USE_OR_YOU_WILL_BE_FIRED; // lol don't worry
    map.mapTypes.set('customTiles', customTilesMapType);
    map.setMapTypeId('customTiles');

    // UNCOMMENT TO TEST:
    // downloadTileListOnClick(map);
  }
}

const GoogleMapComponent = withGoogleMap((props) => (
  <GoogleMap {...props} ref={setMapType} />
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
