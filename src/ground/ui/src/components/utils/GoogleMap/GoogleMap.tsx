import React, { Component } from "react";
import { withGoogleMap, GoogleMap } from "react-google-maps";

import "./GoogleMap.css";
import downloadToBrowser from "utils/downloadToBrowser";
import google, { getLocalImageUrl } from "./google";

function getCustomTilesMapType() {
  return new google.maps.ImageMapType({
    getTileUrl: (coord, zoom) => {
      let url = "";
      try {
        url = getLocalImageUrl(coord, zoom);
        if (!url || url === "") {
          throw new Error();
        }
      } catch (e) {
        // Note: 'v=844' may need to be updated to a new number from time to time.
        url = `https://khms0.googleapis.com/kh?v=844&hl=en-US&x=${coord.x}&y=${coord.y}&z=${zoom}`;
      }

      // UNCOMMENT TO TEST:
      // tileLoaded(coord, zoom);

      return url;
    },
    tileSize: new google.maps.Size(256, 256),
    maxZoom: 20
  });
}

class GoogleMapWrapperComponent extends Component<GoogleMap["props"]> {
  private setMapType = (mapComponent: GoogleMap) => {
    if (mapComponent) {
      // google = window.google;
      let customTilesMapType = getCustomTilesMapType();
      let map =
        mapComponent.context.__SECRET_MAP_DO_NOT_USE_OR_YOU_WILL_BE_FIRED; // lol don't worry
      map.mapTypes.set("customTiles", customTilesMapType);
      // map.setMapTypeId('customTiles');

      // pan smoothly when map center changes
      this.componentDidUpdate = prevProps => {
        if (prevProps.center !== this.props.center && this.props.center) {
          mapComponent.panTo(this.props.center);
        }
      };

      // UNCOMMENT TO TEST:
      // downloadTileListOnClick(map);
    }
  };

  private GoogleMapComponent = withGoogleMap(props => (
    <GoogleMap {...props} ref={this.setMapType} />
  ));

  public render() {
    return (
      <this.GoogleMapComponent
        containerElement={<div style={{ height: `100%` }} />}
        mapElement={<div style={{ height: `100%` }} />}
        {...this.props}
      />
    );
  }
}

export default GoogleMapWrapperComponent;

// For testing only:
var tileBounds: {
  [key: number]: { left: number; right: number; top: number; bottom: number };
} = {};
function tileLoaded(coord: { x: number; y: number }, zoom: number) {
  if (!tileBounds[zoom]) {
    tileBounds[zoom] = {
      left: Infinity,
      right: -Infinity,
      top: Infinity,
      bottom: -Infinity
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
function downloadTileListOnClick(map: any) {
  map.addListener("click", () => {
    let tileUrls: {
      [key: number]: {
        x: number;
        y: number;
        url: string;
      }[];
    } = {};
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
