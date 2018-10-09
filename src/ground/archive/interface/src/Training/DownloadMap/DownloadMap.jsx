import React, { Component } from "react";
import "./DownloadMap.css";
// import GMapCache from "../../Map/GMapCache.jsx";

const METERS_PER_FOOT = 0.3048;
const google = window.google;

class DownloadMap extends Component {
  render() {
    return (
      <div className="DownloadMap">
        <div id="downloadMapMap" ref="download_map" />
        <div id="downloadMapTiles" />
        <div id="downloadMapInfo">
          <button id="downloadMapGetTiles">Get Tiles</button>
          <p id="downloadMapCameraZoom" />
          <p id="downloadMapCameraHeight" />
          <p id="downloadMapMetersPerPixel" />
          <p id="downloadMapOut" />
        </div>
      </div>
    );
  }

  componentDidMount() {
    this.TILE_SIZE = 256;

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
      disableDoubleClickZoom: true,
      mapTypeId: "satellite"
    });

    // this.gmap_cache = new GMapCache();
    let this_local = this;

    // this.map.mapTypes.set(
    //   "offline_gmap",
    //   new google.maps.ImageMapType({
    //     getTileUrl: function(coord, zoom) {
    //       return this_local.getImageTile(coord, zoom);
    //     },
    //     tileSize: new google.maps.Size(256, 256),
    //     name: "LocalMyGmap",
    //     maxZoom: 21,
    //     minZoom: 1
    //   })
    // );

    // this.map.setMapTypeId("offline_gmap");

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

    var practiceField = new google.maps.Marker({
      position: { lat: 34.174982, lng: -118.48152 },
      map: this.map
    });

    var competitionField = new google.maps.Marker({
      position: { lat: 38.145828, lng: -76.427908 },
      map: this.map
    });

    let getTilesButton = document.getElementById("downloadMapGetTiles");
    getTilesButton.addEventListener("click", this.grabTiles.bind(this_local));
  }

  updateAltitudeInfo() {
    let latLng = this.map.getCenter();
    let zoom = this.map.getZoom();
    document.getElementById("downloadMapCameraZoom").innerHTML = zoom;

    document.getElementById(
      "downloadMapCameraHeight"
    ).innerHTML = this.getCameraAltitude(zoom);

    document.getElementById(
      "downloadMapMetersPerPixel"
    ).innerHTML = this.getMetersPerPixel(latLng, zoom);
  }

  getCameraAltitude(zoom) {
    return Math.pow(2.0, 25.0 - zoom) - Math.pow(2.0, 21.4 - zoom);
  }

  getMetersPerPixel(latLng, zoom) {
    return (
      156543.03392 * Math.cos(latLng.lat() * Math.PI / 180) / Math.pow(2, zoom)
    );
  }

  getImageTile(coord, zoom) {
    let img_url = this.gmap_cache.getGmapTileImgSrc(coord, zoom);
    console.log("Getting tile for " + coord + " at zoom " + zoom);

    return img_url;
  }

  grabTiles() {
    let latLng = this.map.getCenter();
    let zoom = this.map.getZoom();
    zoom = 20;

    let scale = 1 << zoom;

    let worldCoordNE = this.project(this.map.getBounds().getNorthEast());
    let worldCoordSW = this.project(this.map.getBounds().getSouthWest());

    let xMin = Math.ceil(worldCoordSW.x * scale / this.TILE_SIZE);
    let xMax = Math.floor(worldCoordNE.x * scale / this.TILE_SIZE);

    let yMin = Math.ceil(worldCoordNE.y * scale / this.TILE_SIZE);
    let yMax = Math.floor(worldCoordSW.y * scale / this.TILE_SIZE);

    console.log(
      "xmin: " + xMin + " xmax: " + xMax + " ymin: " + yMin + " ymax: " + yMax
    );

    let jsonOut = [];

    for (let x = xMin; x < xMax; x++) {
      for (let y = yMin; y < yMax; y++) {
        let coord = new google.maps.Point(x, y);

        let imgTileUrl = this.getImageTile(coord, zoom);

        jsonOut.push({
          url: imgTileUrl,
          altitude: this.getCameraAltitude(zoom),
          zoom: zoom,
          metersPerPixel: this.getMetersPerPixel(latLng, zoom)
        });

        //let tile = document.createElement("img");
        //tile.src = imgTileUrl;
        //document.getElementById("downloadMapTiles").appendChild(tile);
      }
    }

    document.getElementById(
      "downloadMapOut"
    ).innerHTML = JSON.stringify(jsonOut);
  }

  project(latLng) {
    var siny = Math.sin(latLng.lat() * Math.PI / 180);

    // Truncating to 0.9999 effectively limits latitude to 89.189. This is
    // about a third of a tile past the edge of the world tile.
    siny = Math.min(Math.max(siny, -0.9999), 0.9999);

    return new google.maps.Point(
      this.TILE_SIZE * (0.5 + latLng.lng() / 360),
      this.TILE_SIZE * (0.5 - Math.log((1 + siny) / (1 - siny)) / (4 * Math.PI))
    );
  }
}

export default DownloadMap;
