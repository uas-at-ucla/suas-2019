import React, { Component } from "react";
import "./DownloadMap.css";
import GMapCache from "../../Map/GMapCache.jsx";

const METERS_PER_FOOT = 0.3048;
const google = window.google;

class DownloadMap extends Component {
  render() {
    return (
      <div className="DownloadMap">
        <div id="downloadMapMap" ref="download_map" />
        <div id="downloadMapTiles" />
        <div id="downloadMapInfo">
          <p id="downloadMapCameraZoom" />
          <p id="downloadMapCameraHeight" />
          <p id="downloadMapMetersPerPixel" />
        </div>
      </div>
    );
  }

  componentDidMount() {
    let field = {
      lat: 38.145298,
      lng: -76.42861
    };

    this.map = new google.maps.Map(this.refs.download_map, {
      center: field,
      zoom: 11,
      tilt: 0,
      disableDefaultUI: true,
      scrollwheel: true,
      navigationControl: false,
      mapTypeControl: false,
      scaleControl: true,
      draggable: true,
      disableDoubleClickZoom: true
    });

    this.gmap_cache = new GMapCache();
    let this_local = this;

    this.map.mapTypes.set(
      "offline_gmap",
      new google.maps.ImageMapType({
        getTileUrl: function(coord, zoom) {
          return this_local.getImageTile(coord, zoom);
        },
        tileSize: new google.maps.Size(256, 256),
        name: "LocalMyGmap",
        maxZoom: 21,
        minZoom: 12
      })
    );

    this.map.setMapTypeId("offline_gmap");

    this.map.addListener("center_changed", function() {
      console.log("PANNED");
    });

    this.map.addListener("tilesloaded", function() {
      console.log("DONE");
    });

    this.updateAltitudeInfo();
    this.map.addListener("zoom_changed", function() {
      this_local.updateAltitudeInfo();
    });
  }

  updateAltitudeInfo() {
    let latLng = this.map.getCenter();
    let zoom = this.map.getZoom();
    document.getElementById("downloadMapCameraZoom").innerHTML = zoom;

    document.getElementById("downloadMapCameraHeight").innerHTML =
      Math.pow(2.0, 25.0 - zoom) - Math.pow(2.0, 21.4 - zoom);

    document.getElementById("downloadMapMetersPerPixel").innerHTML =
      156543.03392 * Math.cos(latLng.lat() * Math.PI / 180) / Math.pow(2, zoom);
  }

  getImageTile(coord, zoom) {
    let img_url = this.gmap_cache.getGmapTileImgSrc(coord, zoom);

    let tile = document.createElement("img");
    tile.src = img_url;
    //document.getElementById("downloadMapTiles").appendChild(tile);

    return img_url;
  }
}

export default DownloadMap;
