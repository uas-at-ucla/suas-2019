import React, { Component } from 'react';
import GMapCache from '../Map/GMapCache';
import map_style from '../Map/map_style.js';
import './Position.css';


const google = window.google;


// todo: two options:
// 1. Display flight route with position of all photos taken
// 2. When a photo is selected, display its position
class Position extends Component {

  render() {
    return (
      <div className="Position" ref="map"/>
    );
  }


  componentDidMount() {

    // Default field to zoom into.
    let field = {
      //lat: 38.145298,
      //lng: -76.42861
      lat: 34.175048,
      lng: -118.48159
    };


    this.map = new google.maps.Map(this.refs.map, {
      center: field,
      zoom: 17,
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

    this.gmap_cache = new GMapCache();
    let this_local = this;

    this.map.mapTypes.set(
      'offline_gmap',
      new google.maps.ImageMapType({
        getTileUrl: function(coord, zoom) {
          return this_local.gmap_cache.checkTileInSprites(coord, zoom)
               ? this_local.gmap_cache.getLocalTileImgSrc(coord, zoom)
               : this_local.gmap_cache.getGmapTileImgSrc(coord, zoom);
        },
        tileSize: new google.maps.Size(256, 256),
        name: 'LocalMyGmap',
        maxZoom: 21,
        minZoom: 1
      })
    );

    this.map.setMapTypeId('offline_gmap');

  }
}

export default Position;