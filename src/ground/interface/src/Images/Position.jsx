import React, { Component } from 'react';
import GMapCache from '../Map/GMapCache';
import map_style from '../Map/map_style.js';
import './Position.css';
import scriptLoader from 'react-async-script-loader';

const google = window.google;
var photo_position = null;

const INITIAL_LAT = 34.173048;
const INITIAL_LON = -118.48159;

// When a photo is selected, display its position
class Position extends Component {
  constructor(props) {
    super(props);
  }

  render() {
    return (
      <div className="Position" ref="map"/>
    );
  }


  componentWillReceiveProps(nextProps) {
    if (nextProps.isScriptLoaded && !this.props.isScriptLoaded) { // load finished
      if (nextProps.isScriptLoadSucceed) {
        this.initMap();
      }
    }
  }

  componentDidMount() {
    if (google.maps || (this.props.isScriptLoaded && this.props.isScriptLoadSucceed)) {
      this.initMap();
    }
  }

  initMap() {

    // Default field to zoom into.
    let field = {
      lat: INITIAL_LAT,
      lng: INITIAL_LON
    };

    this.map_photo = new google.maps.Map(this.refs.map, {
      center: field,
      zoom: 19,
      tilt: 0,
      disableDefaultUI: true,
      scrollwheel: true,
      navigationControl: false,
      mapTypeControl: false,
      scaleControl: true,
      draggable: true,
      disableDoubleClickZoom: true,
      styles: map_style
    });

    // this.gmap_cache = new GMapCache();
    // let this_local = this;

    // this.map_photo.mapTypes.set(
    //   'offline_gmap',
    //   new google.maps.ImageMapType({
    //     getTileUrl: function(coord, zoom) {
    //       return this_local.gmap_cache.checkTileInSprites(coord, zoom)
    //            ? this_local.gmap_cache.getLocalTileImgSrc(coord, zoom)
    //            : this_local.gmap_cache.getGmapTileImgSrc(coord, zoom);
    //     },
    //     tileSize: new google.maps.Size(256, 256),
    //     name: 'LocalMyGmap',
    //     maxZoom: 21,
    //     minZoom: 1
    //   })
    // );
    // this.map_photo.setMapTypeId('offline_gmap');

    this.map_photo.setMapTypeId('satellite');

    // Declare a single marker
    let placement = {
      lat: 0,
      lng: 0
    };
    let marker_options = {
      map: this.map_photo,
      position: placement
    };
    photo_position = new google.maps.Marker(marker_options);
  }

  // Update the marker posiiton
  componentWillReceiveProps(nextProps) {
    if (nextProps.lat !== this.props.lat &&
        nextProps.lon !== this.props.lon) {
      console.log(nextProps);
      let placement = {
        lat: nextProps.lat,
        lng: nextProps.lon
      };
      photo_position.setPosition(placement);
      this.map_photo.panTo(placement);
    }
  }
}

export default scriptLoader(
    ["https://maps.googleapis.com/maps/api/js?key=AIzaSyBI-Gz_lh3-rKXFwlpElD7pInA60U-iK0c&libraries=visualization"]
)(Position)
