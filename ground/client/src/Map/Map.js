import React, { Component } from 'react';
import './Map.css'
import map_style from './map_style.js'
import * as gmapcache from './gmapcache/script.js';
import drone_marker from '../images/drone_marker.svg';

const METERS_PER_FOOT = 0.3048;
const google = window.google;

class Map extends Component {
  render() {
    return (
      <div className="Map" ref="map"></div>
    )
  }

  componentDidMount() {
    var field = {lat : 38.1470000, lng : -76.4284722};

    this.map = new google.maps.Map(this.refs.map, {
      center : field,
      zoom : 16,
      tilt : 0,
      disableDefaultUI : true,
      scrollwheel : true,
      navigationControl : false,
      mapTypeControl : false,
      scaleControl : true,
      draggable : true,
      styles : map_style
    });

    this.map.mapTypes.set("offline_gmap", new google.maps.ImageMapType({
      getTileUrl: function(coord, zoom) {
        return gmapcache.checkTileInSprites(coord, zoom) ?
          gmapcache.getLocalTileImgSrc(coord, zoom) :
          gmapcache.getGmapTileImgSrc(coord, zoom);
      },
      tileSize: new google.maps.Size(256, 256),
      name: "LocalMyGmap",
      maxZoom: 16,
      minZoom: 0
    }));

    this.map.setMapTypeId("offline_gmap");

    this.drone_marker_icon = {
      path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
      scale: 10,
      rotation: 0,
      anchor: new google.maps.Point(0, 2.5)
    }

    this.drone_marker = new google.maps.Marker({
      map: this.map,
      position: field,
      icon: this.drone_marker_icon
    });

    this.drone_background_marker = new google.maps.Marker({
      map: this.map,
      position: field,
      icon: {
        url: drone_marker,
        anchor: new google.maps.Point(50, 50)
      }
    });

    this.stationary_obstacles = [];
    this.moving_obstacles = [];

    this.follow_drone = true;
    this.map.addListener('dragstart', () => {
      this.follow_drone = false;
    });

    // Always keep drone in the center of view, even after resizing.
    google.maps.event.addListener(this.map, 'bounds_changed', () => {
      if (this.follow_drone) {
        this.pan_to_drone();
      }
    });
  }

  followDrone() {
    this.follow_drone = true;
    this.pan_to_drone();
  }

  update_drone_position(new_lat, new_lng, new_heading) {
    var new_position = new google.maps.LatLng(new_lat, new_lng);

    this.drone_marker.setPosition(new_position);
    this.drone_marker_icon.rotation = new_heading;
    this.drone_marker.setIcon(this.drone_marker_icon);
    this.drone_background_marker.setPosition(new_position);

    if(this.get_distance(new_position, this.map.getCenter()) > 50.0) {
      if (this.follow_drone) {
        this.pan_to_drone();
      }
    }
  }

  pan_to_drone() {
    this.map.panTo(this.drone_marker.getPosition());
  }

  draw_boundary(boundary_pts) {
    var boundary_coordinates = [];

    for (var pt of boundary_pts) {
      boundary_coordinates.push({lat: pt.latitude, lng: pt.longitude});
    }

    var first_pt = boundary_pts[0];
    boundary_coordinates.push({lat: first_pt.latitude, lng: first_pt.longitude});

    var boundary = new google.maps.Polygon({
      path: boundary_coordinates,
      strokeColor: '#00FF00',
      strokeOpacity: 0.7,
      strokeWeight: 3,
      fillColor: '#00FF00',
      fillOpacity: 0.25,
      zIndex: 1
    });

    boundary.setMap(this.map);
  }

  set_stationary_obstacles(obstacles) {
    // Remove any old stationary obstacles that existed before update.
    for(let stationary_obstacle of this.stationary_obstacles) {
      stationary_obstacle.circle.setMap(null);
    }

    this.stationary_obstacles.length = 0;

    // Add in the new stationary obstacles.
    for (let obstacle of obstacles) {
      console.log("making static obstacle");
      let stationary_obstacle = this.make_obstacle_map_object(obstacle);

      this.stationary_obstacles.push(stationary_obstacle);
    }
  }

  set_moving_obstacles(obstacles) {
    // Remove any old moving obstacles.
    for (let moving_obstacle of this.moving_obstacles) {
      moving_obstacle.circle.setMap(null);
    }

    this.moving_obstacles.length = 0;

    // Add in the new moving obstacles.
    for (let obstacle of obstacles) {
      console.log("making moving obstacle");
      let moving_obstacle = this.make_obstacle_map_object(obstacle);

      this.moving_obstacles.push(moving_obstacle);
    }
  }

  make_obstacle_map_object(obstacle) {
    let pos = {lat: obstacle.latitude, lng: obstacle.longitude};
    let radius_feet = obstacle.cylinder_radius || obstacle.sphere_radius

    console.log(obstacle);
    console.log(radius_feet);
    let circle = new google.maps.Circle({
      center: pos,
      map: this.map,
      fillColor: "#FF0000",
      fillOpacity: 0.7,
      strokeWeight: 0,
      radius: radius_feet * METERS_PER_FOOT,
      zIndex: 3
    });

    return {
      circle: circle,
      obstacle: obstacle
    };
  }

  update_moving_obstacles(obstacles) {
    if (obstacles.length !== this.moving_obstacles.length) {
      return false;
    }

    for (let i = 0; i < obstacles.length; i++) {
      let moving_obstacle = this.moving_obstacles[i];

      if (obstacles[i].sphere_radius !==
          moving_obstacle.obstacle.sphere_radius) {
        return false;
      }

      let pos = {
        lat: obstacles[i].latitude,
        lng: obstacles[i].longitude
      };

      moving_obstacle.circle.setCenter(pos);
    }

    return true;
  }

  rad(x) {
    return x * Math.PI / 180;
  }

  get_distance(p1, p2) {
    var R = 6378137; // Earth’s mean radius in meter
    var dLat = this.rad(p2.lat() - p1.lat());
    var dLong = this.rad(p2.lng() - p1.lng());
    var a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(this.rad(p1.lat())) * Math.cos(this.rad(p2.lat())) *
      Math.sin(dLong / 2) * Math.sin(dLong / 2);
    var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    var d = R * c;

    return d; // returns the distance in meter
  }

  refreshMapSize() {
    var center = this.map.getCenter();
    google.maps.event.trigger(this.map, "resize");
    this.map.panTo(center);
  }
};

export default Map;
