import React, { Component } from 'react';
import './Map.css'
import map_style from './map_style.js'
import drone_marker from '../graphics/drone_marker.svg';

const METERS_PER_FOOT = 0.3048;
const google = window.google;

class Map extends Component {
  whenStateChanges = []

  render() {
    return (
      <div className="Map" ref="map"></div>
    )
  }

  stateDidChange(nextProps, stateProp, key) {
    return nextProps[stateProp][key] !== this.props[stateProp][key];
  }

  registerStateDepFunction(stateProp, key, func) {
    func(this.props);

    this.whenStateChanges.push({
      stateProp: stateProp,
      key: key,
      func: func
    })
  }

  componentWillReceiveProps(nextProps) {
    for (let item of this.whenStateChanges) {
      if (this.stateDidChange(nextProps, item.stateProp, item.key)) {
        item.func(nextProps);
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
      scrollwheel : false,
      navigationControl : false,
      mapTypeControl : false,
      scaleControl : true,
      draggable : false,
      styles : map_style
    });

    this.map.mapTypes.set("offline_gmap", new google.maps.ImageMapType({
      getTileUrl: function(coord, zoom) {
        return window.checkTileInSprites(coord, zoom) ?
          window.getLocalTileImgSrc(coord, zoom) :
          window.getGmapTileImgSrc(coord, zoom);
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

    this.mission_commands = [];
    this.commands = [];
    this.command_path = null;
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

    this.map.addListener("dblclick", (e) => {
      if (this.props.onDoubleClick) {
        this.props.onDoubleClick(e.latLng.lat(), e.latLng.lng());
      }
    });

    this.registerStateDepFunction('homeState', 'followDrone',
        this.pan_to_drone_if_following);
    this.registerStateDepFunction('homeState', 'mission',
        this.draw_mission_details);
    this.registerStateDepFunction('homeState', 'commands',
        this.draw_command_path);
    this.registerStateDepFunction('appState', 'telemetry',
        this.update_drone_position);
    this.registerStateDepFunction('appState', 'stationary_obstacles',
        this.set_stationary_obstacles);
    this.registerStateDepFunction('appState', 'moving_obstacles',
        this.refresh_moving_obstacles);
  }

  pan_to_drone_if_following = (props) => {
    if (props.homeState.followDrone) {
      this.pan_to_drone();
      this.map.setZoom(16);
    }
  }

  draw_mission_details = (props) => {
    let mission = props.homeState.mission;

    if (mission) {
      this.draw_mission_commands(mission.mission_commands);
      this.draw_fly_zones(mission.fly_zones);
    }
  }

  draw_command_path = (props) => {
    let commands = props.homeState.commands

    for (let marker of this.commands) {
      marker.setMap(null);
    }

    this.commands.length = 0;

    if (this.command_path) {
      this.command_path.setMap(null);
    }

    let commandPositions = [];

    for (let command of commands) {
      let marker = new google.maps.Marker({
        map: this.map,
        position: command
      });

      let infowindow = new google.maps.InfoWindow({
        content: 'Lat: ' + command.lat + '<br>' +
                 'Lng: ' + command.lng + '<br>' +
                 'Alt: ' + command.alt + ' m'
      });

      marker.addListener('click', () => {
        infowindow.open(this.map, marker);
      });

      google.maps.event.addListener(this.map, "click", () => {
        infowindow.close();
      });

      this.commands.push(marker);

      commandPositions.push({
        lat: command.lat,
        lng: command.lng
      });
    }

    let polyline = new google.maps.Polyline({
      path: commandPositions,
      geodesic: true,
      strokeColor: '#0000FF',
      strokeOpacity: 0.7,
      strokeWeight: 3,
    });

    polyline.setMap(this.map);
    this.command_path = polyline;
  }

  update_drone_position = (props) => {
    let telemetry = props.appState.telemetry;

    if (!telemetry) return;

    let new_position = new google.maps.LatLng(telemetry.gps_lat,
                                              telemetry.gps_lng);

    this.drone_marker.setPosition(new_position);
    this.drone_marker_icon.rotation = telemetry.heading;
    this.drone_marker.setIcon(this.drone_marker_icon);
    this.drone_background_marker.setPosition(new_position);

    if(this.get_distance(new_position, this.map.getCenter()) > 50.0) {
      if (this.props.homeState.followDrone) {
        this.pan_to_drone();
      }
    }
  }

  set_stationary_obstacles = (props) => {
    let obstacles = props.appState.stationary_obstacles;

    // Remove any old stationary obstacles that existed before update.
    for (let stationary_obstacle of this.stationary_obstacles) {
      stationary_obstacle.circle.setMap(null);
    }

    this.stationary_obstacles.length = 0;

    // Add in the new stationary obstacles.
    for (let obstacle of obstacles) {
      let stationary_obstacle = this.make_obstacle_map_object(obstacle);

      this.stationary_obstacles.push(stationary_obstacle);
    }
  }

  refresh_moving_obstacles = (props) => {
    let obstacles = props.appState.moving_obstacles;

    if (!this.update_moving_obstacles(obstacles)) {
      this.set_moving_obstacles(obstacles);
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

  draw_mission_commands(commands) {
    for (let marker of this.mission_commands) {
      marker.setMap(null);
    }

    this.mission_commands.length = 0;

    if(commands == undefined) return;

    for (let command of commands) {
      let coords = {lat: command.latitude, lng: command.longitude};
      let marker = new google.maps.Marker({
        map: this.map,
        position: coords,
        label: {
          fontFamily: 'Fontawesome',
          text: '\uf192'
        }
      });

      coords.alt = command.altitude_msl;

      let infowindow = new google.maps.InfoWindow({
        content: 'Lat: ' + coords.lat + '<br>' +
                 'Lng: ' + coords.lng + '<br>' +
                 'Alt: ' + coords.alt + ' m'
      });

      marker.addListener('click', () => {
        infowindow.open(this.map, marker);
      });

      google.maps.event.addListener(this.map, "click", () => {
        infowindow.close();
      });

      this.mission_commands.push(marker);
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
      let moving_obstacle = this.make_obstacle_map_object(obstacle);

      this.moving_obstacles.push(moving_obstacle);
    }
  }

  make_obstacle_map_object(obstacle) {
    let pos = {lat: obstacle.latitude, lng: obstacle.longitude};
    let radius_feet = obstacle.cylinder_radius || obstacle.sphere_radius

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
               'Radius: ' + radius_feet + ' m'
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
    // Get distance between two coordinates (in meters).

    let R = 6378137; // Earth’s mean radius in meter
    let dLat = this.rad(p2.lat() - p1.lat());
    let dLong = this.rad(p2.lng() - p1.lng());
    let a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(this.rad(p1.lat())) * Math.cos(this.rad(p2.lat())) *
      Math.sin(dLong / 2) * Math.sin(dLong / 2);
    let c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    let d = R * c;

    return d;
  }
};

export default Map;
