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

  stateDidChange(nextProps, stateProp, key) {
    return nextProps[stateProp][key] !== this.props[stateProp][key];
  }

  componentWillReceiveProps(nextProps) {
    if (this.stateDidChange(nextProps, 'homeState', 'followDrone')) {
      if (nextProps.homeState.followDrone) {
        this.pan_to_drone();
        this.map.setZoom(16);
      }
    }

    if (this.stateDidChange(nextProps, 'homeState', 'mission')) {
      let mission = nextProps.homeState.mission;
      if (mission) {
        this.draw_mission_waypoints(mission.mission_waypoints);
        this.draw_fly_zones(mission.fly_zones);
      }
    }

    if (this.stateDidChange(nextProps, 'homeState', 'waypoints')) {
      this.draw_waypoint_path(nextProps.homeState.waypoints);
    }

    if (this.stateDidChange(nextProps, 'appState', 'telemetry')) {
      let telemetry = nextProps.appState.telemetry;
      this.update_drone_position(telemetry.gps_lat, telemetry.gps_lng, telemetry.heading);
    }

    if (this.stateDidChange(nextProps, 'appState', 'stationary_obstacles')) {
      let obstacles = nextProps.appState.stationary_obstacles;
      this.set_stationary_obstacles(obstacles);
    }

    if (this.stateDidChange(nextProps, 'appState', 'moving_obstacles')) {
      let obstacles = nextProps.appState.moving_obstacles;
      if (!this.update_moving_obstacles(obstacles)) {
        console.log('New moving obstacles received!');
        this.set_moving_obstacles(obstacles);
      }
    }
  }

  componentDidMount() {
    let field = {lat : 38.1470000, lng : -76.4284722};

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

    this.mission_waypoints = [];
    this.waypoints = [];
    this.waypoint_path = null;
    this.fly_zones = [];

    this.map.addListener('dragstart', () => {
      this.props.setHomeState({followDrone: false});
    });

    // Always keep drone in the center of view, even after resizing.
    google.maps.event.addListener(this.map, 'bounds_changed', () => {
      if (this.props.homeState.followDrone) {
        this.pan_to_drone();
      }
    });
  }

  update_drone_position(new_lat, new_lng, new_heading) {
    let new_position = new google.maps.LatLng(new_lat, new_lng);

    this.drone_marker.setPosition(new_position);
    this.drone_marker_icon.rotation = new_heading;
    this.drone_marker.setIcon(this.drone_marker_icon);
    this.drone_background_marker.setPosition(new_position);

    if(this.get_distance(new_position, this.map.getCenter()) > 50.0) {
      if (this.props.homeState.followDrone) {
        this.pan_to_drone();
      }
    }
  }

  pan_to_drone() {
    this.map.panTo(this.drone_marker.getPosition());
  }

  draw_fly_zones(fly_zones) {
    for (let polygon of this.fly_zones) {
      polygon.setMap(null);
    }
    this.fly_zones.length = 0;

    for (let fly_zone of fly_zones) {
      let boundary_coordinates = [];

      for (let pt of fly_zone.boundary_pts) {
        boundary_coordinates.push({lat: pt.latitude, lng: pt.longitude});
      }

      // let first_pt = boundary_pts[0];
      // boundary_coordinates.push({lat: first_pt.latitude, lng: first_pt.longitude});

      let polygon = new google.maps.Polygon({
        path: boundary_coordinates,
        strokeColor: '#00FF00',
        strokeOpacity: 0.7,
        strokeWeight: 3,
        fillColor: '#00FF00',
        fillOpacity: 0.25,
      });

      polygon.setMap(this.map);
      this.fly_zones.push(polygon);
    }
  }

  draw_mission_waypoints(waypoints) {
    for (let marker of this.mission_waypoints) {
      marker.setMap(null);
    }
    this.mission_waypoints.length = 0;

    for (let waypoint of waypoints) {
      let coords = {lat: waypoint.latitude, lng: waypoint.longitude};
      let marker = new google.maps.Marker({
        map: this.map,
        position: coords,
        label: {
          fontFamily: 'Fontawesome',
          text: '\uf192'
        }
      });
      coords.alt = waypoint.altitude_msl;
      let infowindow = new google.maps.InfoWindow({
      content: 'Lat: ' + coords.lat + '<br>' + 
               'Lng: ' + coords.lng + '<br>' +
               'Alt: ' + coords.alt + ' ft'
      });
      marker.addListener('click', () => {
        infowindow.open(this.map, marker);
      });
      google.maps.event.addListener(this.map, "click", () => {
        infowindow.close();
      });
      this.mission_waypoints.push(marker);
    }
  }

  draw_waypoint_path(waypoints) {
    for (let marker of this.waypoints) {
      marker.setMap(null);
    }
    this.waypoints.length = 0;
    if (this.waypoint_path) {
      this.waypoint_path.setMap(null);
    }

    let polyline = new google.maps.Polyline({
      path: [this.drone_marker.getPosition()].concat(waypoints),
      geodesic: true,
      strokeColor: '#0000FF',
      strokeOpacity: 0.7,
      strokeWeight: 3,
    });
    polyline.setMap(this.map);
    this.waypoint_path = polyline

    for (let waypoint of waypoints) {
      if (waypoint.fromMission) {
        continue;
      }
      let marker = new google.maps.Marker({
        map: this.map,
        position: waypoint
      });
      let infowindow = new google.maps.InfoWindow({
      content: 'Lat: ' + waypoint.lat + '<br>' + 
               'Lng: ' + waypoint.lng + '<br>' +
               'Alt: ' + waypoint.alt + ' ft'
      });
      marker.addListener('click', () => {
        infowindow.open(this.map, marker);
      });
      google.maps.event.addListener(this.map, "click", () => {
        infowindow.close();
      });
      this.waypoints.push(marker);
    }
  }

  set_stationary_obstacles(obstacles) {
    // Remove any old stationary obstacles that existed before update.
    for (let stationary_obstacle of this.stationary_obstacles) {
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
    let infowindow = new google.maps.InfoWindow({
      content: 'Lat: ' + obstacle.latitude + '<br>' + 
               'Lng: ' + obstacle.longitude + '<br>' +
               'Radius: ' + radius_feet + ' ft'
    });
    google.maps.event.addListener(circle, 'click', () => {
      infowindow.setPosition(circle.getCenter());
      infowindow.open(this.map);
    });
    google.maps.event.addListener(this.map, "click", () => {
      infowindow.close();
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
    let R = 6378137; // Earth’s mean radius in meter
    let dLat = this.rad(p2.lat() - p1.lat());
    let dLong = this.rad(p2.lng() - p1.lng());
    let a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(this.rad(p1.lat())) * Math.cos(this.rad(p2.lat())) *
      Math.sin(dLong / 2) * Math.sin(dLong / 2);
    let c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    let d = R * c;

    return d; // returns the distance in meter
  }
};

export default Map;
