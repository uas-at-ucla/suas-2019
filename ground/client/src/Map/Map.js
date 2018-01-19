import React, { Component } from "react";
import "./Map.css";
import map_style from "./map_style.js";
import drone_marker from "../graphics/drone_marker.svg";

const METERS_PER_FOOT = 0.3048;
const google = window.google;

class Map extends Component {
  whenStateChanges = [];

  render() {
    return <div className="Map" ref="map" />;
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
    });
  }

  componentWillReceiveProps(nextProps) {
    for (let item of this.whenStateChanges) {
      if (this.stateDidChange(nextProps, item.stateProp, item.key)) {
        item.func(nextProps);
      }
    }
  }

  componentDidMount() {
    let field = {
      lat: 38.145298,
      lng: -76.42861
    };

    this.map = new google.maps.Map(this.refs.map, {
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
      styles: map_style
    });

    this.map.mapTypes.set(
      "offline_gmap",
      new google.maps.ImageMapType({
        getTileUrl: function(coord, zoom) {
          return window.checkTileInSprites(coord, zoom)
            ? window.getLocalTileImgSrc(coord, zoom)
            : window.getGmapTileImgSrc(coord, zoom);
        },
        tileSize: new google.maps.Size(256, 256),
        name: "LocalMyGmap",
        maxZoom: 19,
        minZoom: 0
      })
    );

    this.map.setMapTypeId("offline_gmap");

    this.drone_marker_icon = {
      path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
      strokeColor: "#FFFFFF",
      strokeOpacity: 0.8,
      strokeWeight: 3,
      fillColor: "#0000FF",
      fillOpacity: 0.5,
      scale: 7,
      rotation: 0,
      anchor: new google.maps.Point(0, 2.5)
    };

    this.drone_marker = new google.maps.Marker({
      map: this.map,
      position: field,
      icon: this.drone_marker_icon
    });

    this.stationary_obstacles = [];
    this.moving_obstacles = [];

    this.mission_points = {
      waypoints: [],
      person: null,
      off_axis_object: null,
      home: null,
      air_drop: null
    };
    this.search_grid = null;
    this.commands = [];
    this.command_path = null;
    this.fly_zones = [];

    this.map.addListener("dragstart", () => {
      this.props.setHomeState({ followDrone: false });
    });

    // Always keep drone in the center of view, even after resizing.
    google.maps.event.addListener(this.map, "bounds_changed", () => {
      if (this.props.homeState.followDrone) {
        this.pan_to_drone();
      }
    });

    this.map.addListener("dblclick", e => {
      this.addGotoCommand(e.latLng.lat(), e.latLng.lng(), 80);
    });

    this.registerStateDepFunction(
      "homeState",
      "followDrone",
      this.pan_to_drone_if_following
    );
    this.registerStateDepFunction(
      "homeState",
      "mission",
      this.draw_mission_details
    );
    this.registerStateDepFunction(
      "homeState",
      "commands",
      this.draw_command_path
    );
    this.registerStateDepFunction(
      "homeState",
      "focusedCommand",
      this.focus_on_command
    );
    this.registerStateDepFunction(
      "appState",
      "telemetry",
      this.update_drone_position
    );
    this.registerStateDepFunction(
      "appState",
      "stationary_obstacles",
      this.set_stationary_obstacles
    );
    this.registerStateDepFunction(
      "appState",
      "moving_obstacles",
      this.refresh_moving_obstacles
    );
  }

  pan_to_drone_if_following = props => {
    if (props.homeState.followDrone) {
      this.pan_to_drone();
      this.map.setZoom(16);
    }
  };

  draw_mission_details = props => {
    let mission = props.homeState.mission;

    if (mission) {
      this.draw_boundaries(mission.fly_zones);
      this.draw_target_search_area(mission.search_grid_points);
      this.draw_mission_waypoints(mission.mission_waypoints);
      this.draw_mission_point("person", mission.emergent_last_known_pos);
      this.draw_mission_point("off_axis_object", mission.off_axis_odlc_pos);
      this.draw_mission_point("home", mission.home_pos);
      this.draw_mission_point("air_drop", mission.air_drop_pos);
    }
  };

  draw_command_path = props => {
    let commands = props.homeState.commands;

    if (props.homeState.dontRedrawCommands) {
      this.props.setHomeState({ dontRedrawCommands: false });
      this.update_commands(commands);
      return;
    }

    for (let command_object of this.commands) {
      if (command_object) {
        command_object.marker.setMap(null);
      }
    }

    this.commands.length = 0;

    if (this.command_path) {
      this.command_path.setMap(null);
    }

    let commandPositions = [];

    for (let i = 0; i < commands.length; i++) {
      let command = commands[i];

      if (command.mission_point && command.mission_point.marker.getMap()) {
        this.update_mission_point(command, i);
        this.commands.push(null);
      } else {
        let marker = new google.maps.Marker({
          map: this.map,
          position: command.options
        });

        let title = i + 1 + ") " + command.options.command_type;
        if (command.name) {
          title = title + ": " + command.name;
        }

        let infowindow = new google.maps.InfoWindow({
          content:
            '<div id="command_infowindow_' +
            i +
            '">' +
            "<h6>" +
            title +
            "</h6>" +
            "Lat: " +
            command.options.lat +
            "<br>" +
            "Lng: " +
            command.options.lng +
            "<br>" +
            "Alt: " +
            command.options.alt +
            " m<br>" +
            '<button class="remove_command btn btn-sm btn-outline-danger">' +
            "Remove</button></div>"
        });

        let setupListeners = () => {
          let div = document.getElementById("command_infowindow_" + i);
          if (div) {
            let remove_btn = div.getElementsByClassName("remove_command")[0];
            remove_btn.onclick = () => {
              let commands = this.props.homeState.commands.slice();
              commands.splice(i, 1);
              this.props.setHomeState({ commands: commands });
            };
          }
        };

        marker.addListener("click", () => {
          infowindow.open(this.map, marker);
          this.commands[i].onInfoOpened();
        });

        google.maps.event.addListener(this.map, "click", () => {
          infowindow.close();
        });

        this.commands.push({
          marker: marker,
          infowindow: infowindow,
          onInfoChanged: setupListeners,
          onInfoOpened: setupListeners
        });
      }

      commandPositions.push({
        lat: command.options.lat,
        lng: command.options.lng
      });
    }

    let polyline = new google.maps.Polyline({
      path: commandPositions,
      geodesic: true,
      strokeColor: "#0000FF",
      strokeOpacity: 0.7,
      strokeWeight: 3
    });

    polyline.setMap(this.map);
    this.command_path = polyline;
  };

  focus_on_command = props => {
    if (props.homeState.focusedCommand !== null) {
      this.props.setHomeState({
        followDrone: false,
        focusedCommand: null
      });
      let point =
        this.commands[props.homeState.focusedCommand] ||
        props.homeState.commands[props.homeState.focusedCommand].mission_point;
      this.map.panTo(point.marker.getPosition());
      point.marker.setAnimation(google.maps.Animation.BOUNCE);
      setTimeout(() => point.marker.setAnimation(null), 1400);
    }
  };

  update_drone_position = props => {
    let telemetry = props.appState.telemetry;

    if (!telemetry) return;

    let new_position = new google.maps.LatLng(
      telemetry.gps_lat,
      telemetry.gps_lng
    );

    this.drone_marker.setPosition(new_position);
    this.drone_marker_icon.rotation = telemetry.heading;
    this.drone_marker.setIcon(this.drone_marker_icon);

    if (this.get_distance(new_position, this.map.getCenter()) > 50.0) {
      if (this.props.homeState.followDrone) {
        this.pan_to_drone();
      }
    }
  };

  set_stationary_obstacles = props => {
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
  };

  refresh_moving_obstacles = props => {
    let obstacles = props.appState.moving_obstacles;

    if (!this.update_moving_obstacles(obstacles)) {
      this.set_moving_obstacles(obstacles);
    }
  };

  addGotoCommand(lat, lng, alt, name, mission_point) {
    let commands = this.props.homeState.commands.slice();

    commands.push({
      name: name || "",
      mission_point: mission_point,
      options: {
        command_type: "goto",
        lat: lat,
        lng: lng,
        alt: alt
      }
    });

    this.props.setHomeState({ commands: commands });
  }

  pan_to_drone() {
    this.map.panTo(this.drone_marker.getPosition());
  }

  update_commands(commands) {
    for (let i = 0; i < commands.length; i++) {
      let command = commands[i];

      if (command.mission_point && command.mission_point.marker.getMap()) {
        this.update_mission_point(command, i);
      } else if (this.commands[i]) {
        let title = i + 1 + ") " + command.options.command_type;
        if (command.name) {
          title = title + ": " + command.name;
        }
        this.commands[i].infowindow.setContent(
          '<div id="command_infowindow_' +
            i +
            '">' +
            "<h6>" +
            title +
            "</h6>" +
            "Lat: " +
            command.options.lat +
            "<br>" +
            "Lng: " +
            command.options.lng +
            "<br>" +
            "Alt: " +
            command.options.alt +
            " m<br>" +
            '<button class="remove_command btn btn-sm btn-outline-danger">' +
            "Remove</button></div>"
        );
        this.commands[i].onInfoChanged();
      }
    }
  }

  mission_point_title(mission_point_key, pos) {
    switch (mission_point_key) {
      case "waypoints":
        return "Mission Waypoint " + pos.order;
      case "person":
        return "Person";
      case "off_axis_object":
        return "Off-Axis Object";
      case "home":
        return "Home";
      case "air_drop":
        return "Air Drop";
      default:
        return null;
    }
  }

  mission_point_label(mission_point_key) {
    switch (mission_point_key) {
      case "waypoints":
        return {
          fontFamily: "Fontawesome",
          text: "\uf192",
          fontSize: "15px"
        };
      case "person":
        return {
          fontFamily: "Fontawesome",
          text: "\uf183",
          fontSize: "20px"
        };
      case "off_axis_object":
        return {
          fontFamily: "Fontawesome",
          text: "\uf030",
          fontSize: "14px"
        };
      case "home":
        return {
          fontFamily: "Fontawesome",
          text: "\uf015",
          fontSize: "18px"
        };
      case "air_drop":
        return {
          fontFamily: "Fontawesome",
          text: "\uf1e2",
          fontSize: "15px"
        };
      default:
        return null;
    }
  }

  draw_boundaries(fly_zones) {
    let bigRect = [
      { lat: 38.17, lng: -76.47 },
      { lat: 38.12, lng: -76.47 },
      { lat: 38.12, lng: -76.39 },
      { lat: 38.17, lng: -76.39 }
    ];

    for (let polygon of this.fly_zones) {
      polygon.setMap(null);
    }

    this.fly_zones.length = 0;

    for (let fly_zone of fly_zones) {
      let boundary_coordinates = [];

      fly_zone.boundary_pts.sort((a, b) => a.order - b.order);

      for (let pt of fly_zone.boundary_pts) {
        boundary_coordinates.push({ lat: pt.latitude, lng: pt.longitude });
      }

      let paths = [boundary_coordinates];
      if (this.polygonIsClockwise(boundary_coordinates)) {
        paths.push(bigRect);
      } else {
        paths.push(bigRect.slice().reverse());
      }

      let polygon = new google.maps.Polygon({
        paths: paths,
        strokeColor: "#FF0000",
        strokeOpacity: 0.4,
        strokeWeight: 2,
        fillColor: "#FF0000",
        fillOpacity: 0.2,
        clickable: false
      });

      polygon.setMap(this.map);
      this.fly_zones.push(polygon);
    }
  }

  polygonIsClockwise(vertices) {
    // Determine whether vertices are clockwise/counterclockwise using the
    // sign of the output of using the shoelace formula.
    let area = 0;

    for (let i = 0; i < vertices.length; i++) {
      let j = (i + 1) % vertices.length;
      area += vertices[i].lng * vertices[j].lat;
      area -= vertices[j].lng * vertices[i].lat;
    }

    return area < 0;
  }

  draw_mission_waypoints(waypoints) {
    for (let waypoint of this.mission_points.waypoints) {
      waypoint.marker.setMap(null);
    }
    this.mission_points.waypoints.length = 0;

    if (waypoints == undefined) return;

    let positions = [];

    for (let pos of waypoints) {
      let mission_point = this.make_mission_marker("waypoints", pos);
      this.mission_points.waypoints.push(mission_point);
      positions.push({
        lat: pos.latitude,
        lng: pos.longitude
      });
    }

    let polyline = new google.maps.Polyline({
      path: positions,
      geodesic: true,
      strokeColor: "#00FFFF",
      strokeOpacity: 0,
      strokeWeight: 3,
      icons: [
        {
          icon: {
            path: "M 0,-1 0,1",
            strokeOpacity: 0.7,
            scale: 4
          },
          offset: "0",
          repeat: "20px"
        }
      ]
    });

    polyline.setMap(this.map);
    // this.command_path = polyline;
  }

  draw_mission_point(mission_point_key, pos) {
    if (this.mission_points[mission_point_key]) {
      this.mission_points[mission_point_key].marker.setMap(null);
    }

    let mission_point = this.make_mission_marker(mission_point_key, pos);
    this.mission_points[mission_point_key] = mission_point;
  }

  update_mission_point(command, index) {
    let div = document.createElement("div");
    div.innerHTML = command.mission_point.infowindow.getContent();
    let command_info = index + 1 + ") " + command.options.command_type + ": ";
    div.getElementsByClassName("command_info")[0].textContent = command_info;
    command.mission_point.infowindow.setContent(div.innerHTML);
    command.mission_point.onInfoChanged();
  }

  draw_target_search_area(points) {
    if (this.search_grid) {
      this.search_grid.setMap(null);
    }

    let boundary_coordinates = [];

    points.sort((a, b) => a.order - b.order);

    for (let pt of points) {
      boundary_coordinates.push({ lat: pt.latitude, lng: pt.longitude });
    }

    let polygon = new google.maps.Polygon({
      path: boundary_coordinates,
      strokeColor: "#00FF00",
      strokeOpacity: 0.4,
      strokeWeight: 1,
      fillColor: "#00FF00",
      fillOpacity: 0.2,
      clickable: false
    });

    polygon.setMap(this.map);
    this.search_grid = polygon;
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

  make_mission_marker(mission_point_key, pos) {
    let coords = { lat: pos.latitude, lng: pos.longitude };
    let title = this.mission_point_title(mission_point_key, pos);
    let label = this.mission_point_label(mission_point_key);

    let marker_options = {
      map: this.map,
      position: coords,
      path: google.maps.SymbolPath.circle,
      strokeColor: "#FFFFFF",
      strokeOpacity: 0.8,
      strokeWeight: 3
    };
    if (label) {
      marker_options.label = label;
    }

    let marker = new google.maps.Marker(marker_options);

    let id = mission_point_key;
    if (pos.order) {
      id = id + "_" + pos.order;
    }

    let info =
      '<div id="mission_point_infowindow_' +
      id +
      '">' +
      '<h6 class="infowindow_title">' +
      '<span class="command_info"></span>' +
      title +
      "</h6>" +
      "Lat: " +
      coords.lat +
      "<br>" +
      "Lng: " +
      coords.lng;
    if (pos.altitude_msl) {
      coords.alt = pos.altitude_msl * METERS_PER_FOOT;
      info = info + "<br>" + "Alt: " + coords.alt + " m";
    }

    info =
      info +
      "<br>" +
      "<button " +
      'class="add_point_to_plan btn btn-sm btn-outline-success">' +
      "Add to Plan" +
      "</button>" +
      "<button " +
      "hidden " +
      'class="remove_point_from_plan btn btn-sm btn-outline-danger">' +
      "Remove from Plan" +
      "</button>" +
      "</div>";

    let infowindow = new google.maps.InfoWindow({
      content: info
    });

    let mission_point = {
      marker: marker,
      infowindow: infowindow
    };

    let setupListeners = () => {
      let div = document.getElementById("mission_point_infowindow_" + id);
      if (div) {
        let add_btn = div.getElementsByClassName("add_point_to_plan")[0];
        let remove_btn = div.getElementsByClassName(
          "remove_point_from_plan"
        )[0];
        add_btn.onclick = () => {
          add_btn.setAttribute("hidden", "hidden");
          remove_btn.removeAttribute("hidden");
          infowindow.setContent(div.outerHTML);
          mission_point.onInfoChanged();
          this.addGotoCommand(
            coords.lat,
            coords.lng,
            coords.alt || 80,
            title,
            mission_point
          );
        };
        remove_btn.onclick = () => {
          let command_index = this.props.homeState.commands.findIndex(
            el => el.mission_point === mission_point
          );
          if (command_index !== -1) {
            remove_btn.setAttribute("hidden", "hidden");
            add_btn.removeAttribute("hidden");
            let command_info = div.getElementsByClassName("command_info")[0];
            command_info.textContent = "";
            infowindow.setContent(div.outerHTML);
            mission_point.onInfoChanged();
            let commands = this.props.homeState.commands.slice();
            commands.splice(command_index, 1);
            this.props.setHomeState({ commands: commands });
          }
        };
      }
    };

    mission_point.onInfoChanged = setupListeners;
    mission_point.onInfoOpened = setupListeners;

    marker.addListener("click", () => {
      infowindow.open(this.map, marker);
      mission_point.onInfoOpened();
    });

    google.maps.event.addListener(this.map, "click", () => {
      infowindow.close();
    });

    return mission_point;
  }

  make_obstacle_map_object(obstacle) {
    let pos = { lat: obstacle.latitude, lng: obstacle.longitude };
    let radius_feet = obstacle.cylinder_radius || obstacle.sphere_radius;
    let radius = radius_feet * METERS_PER_FOOT;

    let circle = new google.maps.Circle({
      center: pos,
      map: this.map,
      fillColor: "#FF0000",
      fillOpacity: 0.7,
      strokeWeight: 0,
      radius: radius,
      zIndex: 3
    });

    let infowindow = new google.maps.InfoWindow({
      content: this.make_obstacle_info(obstacle)
    });

    google.maps.event.addListener(circle, "click", () => {
      infowindow.setPosition(circle.getCenter());
      infowindow.open(this.map);
    });

    google.maps.event.addListener(this.map, "click", () => {
      infowindow.close();
    });

    return {
      circle: circle,
      obstacle: obstacle,
      infowindow: infowindow
    };
  }

  make_obstacle_info(obstacle) {
    let radius_feet = obstacle.cylinder_radius || obstacle.sphere_radius;
    let radius = radius_feet * METERS_PER_FOOT;
    let info =
      "<h6>" +
      (obstacle.cylinder_radius ? "Stationary" : "Moving") +
      " Obstacle</h6>" +
      "Lat: " +
      obstacle.latitude +
      "<br>" +
      "Lng: " +
      obstacle.longitude +
      "<br>" +
      "Radius: " +
      radius +
      " m";
    if (obstacle.cylinder_height) {
      info =
        info +
        "<br>Height: " +
        obstacle.cylinder_height * METERS_PER_FOOT +
        " m";
    } else if (obstacle.altitude_msl) {
      info =
        info + "<br>Alt: " + obstacle.altitude_msl * METERS_PER_FOOT + " m";
    }
    return info;
  }

  update_moving_obstacles(obstacles) {
    if (obstacles.length !== this.moving_obstacles.length) {
      return false;
    }

    for (let i = 0; i < obstacles.length; i++) {
      let moving_obstacle = this.moving_obstacles[i];

      if (
        obstacles[i].sphere_radius !== moving_obstacle.obstacle.sphere_radius
      ) {
        return false;
      }

      let pos = {
        lat: obstacles[i].latitude,
        lng: obstacles[i].longitude
      };

      moving_obstacle.circle.setCenter(pos);
      moving_obstacle.infowindow.setPosition(pos);
      moving_obstacle.infowindow.setContent(
        this.make_obstacle_info(obstacles[i])
      );
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
    let a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(this.rad(p1.lat())) *
        Math.cos(this.rad(p2.lat())) *
        Math.sin(dLong / 2) *
        Math.sin(dLong / 2);
    let c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    let d = R * c;

    return d;
  }
}

export default Map;

