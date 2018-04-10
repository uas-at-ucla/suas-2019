import React, { Component } from 'react';
import ReactDOMServer from 'react-dom/server';
import './Map.css';
import GMapCache from './GMapCache.jsx';
import map_style from './map_style.js';

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
    this.last_commands = null;

    // Default field to zoom into.
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
        minZoom: 12
      })
    );

    this.map.setMapTypeId('offline_gmap');

    this.drone_marker_icon = {
      path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
      strokeColor: '#FFFFFF',
      strokeOpacity: 0.8,
      strokeWeight: 3,
      fillColor: '#0000FF',
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
    this.waypoint_path = null;
    this.search_grid = null;
    this.commands = [];
    this.commands_path = [];
    this.fly_zones = [];

    this.map.addListener('dragstart', () => {
      this.props.setAppState({ followDrone: false });
    });

    this.map.addListener('dblclick', e => {
      this.add_goto_command(e.latLng.lat(), e.latLng.lng(), 30);
    });

    this.registerStateDepFunction(
      'appState',
      'followDrone',
      this.pan_to_drone_if_following
    );
    this.registerStateDepFunction(
      'homeState',
      'mission',
      this.draw_mission_details
    );
    this.registerStateDepFunction(
      'homeState',
      'commands',
      this.draw_command_path
    );
    this.registerStateDepFunction(
      'homeState',
      'focusedCommand',
      this.focus_on_command
    );
    this.registerStateDepFunction(
      'appState',
      'telemetry',
      this.update_drone_position
    );
    this.registerStateDepFunction(
      'appState',
      'stationary_obstacles',
      this.set_stationary_obstacles
    );
    this.registerStateDepFunction(
      'appState',
      'moving_obstacles',
      this.refresh_moving_obstacles
    );
  }

  draw_mission_details = props => {
    let mission = props.homeState.mission;

    if (mission) {
      this.draw_boundaries(mission.fly_zones);
      this.draw_target_search_area(mission.search_grid_points);
      this.draw_mission_waypoints(mission.mission_waypoints);
      this.draw_mission_point('person', mission.emergent_last_known_pos);
      this.draw_mission_point('off_axis_object', mission.off_axis_odlc_pos);
      this.draw_mission_point('home', mission.home_pos);
      this.draw_mission_point('air_drop', mission.air_drop_pos);
    }
  };

  draw_command_path = props => {
    let commands = props.homeState.commands;

    let startIndex = 0;
    let endIndex = commands.length-1;
    if (props.homeState.changedCommands !== null) {
      startIndex = props.homeState.changedCommands.startIndex;
      endIndex = props.homeState.changedCommands.endIndex
    }

    // Reset command annotations on map.
    for (let i = startIndex; i <= endIndex; i++) {
      if (this.commands[i]) {
        this.commands[i].marker.setMap(null);
      }
      if (this.commands_path[i]) {
        this.commands_path[i].setMap(null);
      }
    }
    if (this.commands_path[endIndex+1]) {
      this.commands_path[endIndex+1].setMap(null);
    }

    this.commands.length = commands.length;
    this.commands_path.length = commands.length;
    let commandPositions = [];

    // Draw the command path.
    const line_symbol = {
      path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
      strokeColor: '#0000FF'
    };

    let last_pos = null;

    for (let i = startIndex-1; i <= endIndex+1; i++) {
      if (!(0 <= i && i < commands.length)) {
        continue;
      }

      let command = commands[i];
      let type = command.type;
      let fields = command[type];

      if (startIndex <= i && i <= endIndex) {
        if (
          command.mission_point &&
          command.mission_point.marker &&
          command.mission_point.marker.getMap()
        ) {
          this.update_mission_point(command, i);
          this.commands[i] = null;
        } else if (fields.latitude && fields.longitude) {
          let marker = new google.maps.Marker({
            map: this.map,
            position: { lat: fields.latitude, lng: fields.longitude }
          });

          let infowindow = new google.maps.InfoWindow({
            content: this.command_info(i, type, fields)
          });

          let setup_listeners = () => {
            let div = document.getElementById('command_infowindow_' + i);
            if (div) {
              let remove_btn = div.getElementsByClassName('remove_command')[0];
              remove_btn.onclick = () => {
                let commands = this.props.homeState.commands.slice();
                commands.splice(i, 1);
                this.props.setHomeState({
                  commands: commands,
                  changedCommands: {startIndex: i, endIndex: commands.length}
                });
              };
              let drag_btn = div.getElementsByClassName('drag_command')[0];
              let place_btn = div.getElementsByClassName('place_command')[0];
              if (command.draggable) {
                drag_btn.setAttribute('hidden', 'hidden');
                place_btn.removeAttribute('hidden');
              }
              drag_btn.onclick = () => {
                command.draggable = true;
                this.commands[i].marker.setDraggable(true);
                this.commands[i].marker.setLabel({
                  fontFamily: 'Fontawesome',
                  text: '\uf256',
                  fontSize: '15px'
                });
                drag_btn.setAttribute('hidden', 'hidden');
                place_btn.removeAttribute('hidden');
              };
              place_btn.onclick = () => {
                command.draggable = false;
                this.commands[i].marker.setDraggable(false);
                this.commands[i].marker.setLabel(null);
                place_btn.setAttribute('hidden', 'hidden');
                drag_btn.removeAttribute('hidden');
              };
            }
          };

          if (command.draggable) {
            marker.setDraggable(true);
            marker.setLabel({
              fontFamily: 'Fontawesome',
              text: '\uf256',
              fontSize: '15px'
            });
          }

          marker.addListener('click', () => {
            infowindow.open(this.map, marker);
            this.commands[i].onInfoOpened();
          });

          marker.addListener('dragend', () => {
            let commands = this.props.homeState.commands.slice();
            fields.latitude = marker.getPosition().lat();
            fields.longitude = marker.getPosition().lng();

            this.props.setHomeState({
              commands: commands,
              changedCommands: {startIndex: i, endIndex: i}
            });
          });

          google.maps.event.addListener(this.map, 'click', () => {
            infowindow.close();
          });

          this.commands[i] = {
            marker: marker,
            infowindow: infowindow,
            onInfoChanged: setup_listeners,
            onInfoOpened: setup_listeners
          };
        } else {
          this.commands[i] = null;
        }
      }

      if (fields.latitude && fields.longitude) {
        let pos = { lat: fields.latitude, lng: fields.longitude };
        if (last_pos != null) {
          // Create the polyline and add the symbol via the 'icons' property.
          this.commands_path[i] =
            new google.maps.Polyline({
              path: [
                { lat: last_pos.lat, lng: last_pos.lng },
                { lat: pos.lat, lng: pos.lng }
              ],
              icons: [
                {
                  icon: line_symbol,
                  offset: '100%'
                }
              ],
              strokeColor: '#0000FF',
              zIndex: 10,
              map: this.map
            });
        }
        last_pos = pos;
      } else {
        this.commands_path[i] = null;
      } /*else {
        for (let pt of command.options.boundary_pts) {
          //TODO: generate search path
          commandPositions.push({
            lat: pt.lat,
            lng: pt.lng
          });
        }
        commandPositions.push({
          lat: command.options.boundary_pts[0].lat,
          lng: command.options.boundary_pts[0].lng
        });
      }*/
    }
  };

  focus_on_command = props => {
    if (props.homeState.focusedCommand !== null) {
      this.props.setAppState({ followDrone: false });
      this.props.setHomeState({ focusedCommand: null });
      let point =
        this.commands[props.homeState.focusedCommand] ||
        props.homeState.commands[props.homeState.focusedCommand].mission_point;
      if (point && point.marker) {
        this.map.panTo(point.marker.getPosition());
        point.marker.setAnimation(google.maps.Animation.BOUNCE);
        setTimeout(() => point.marker.setAnimation(null), 1400);
      }
    }
  };

  update_drone_position = props => {
    let telemetry = props.appState.telemetry;

    if (!telemetry) return;

    let new_position = new google.maps.LatLng(
      telemetry.sensors.latitude,
      telemetry.sensors.longitude
    );

    this.drone_marker.setPosition(new_position);
    this.drone_marker_icon.rotation = telemetry.sensors.heading;
    this.drone_marker.setIcon(this.drone_marker_icon);

    if (this.get_distance(new_position, this.map.getCenter()) > 50.0) {
      if (this.props.appState.followDrone) {
        this.pan_to_drone();
      }
    }
  };

  add_goto_command(lat, lng, alt, name, mission_point) {
    let command = this.props.makeCommand('GotoCommand', {
      latitude: lat,
      longitude: lng,
      altitude: alt
    });
    command.name = name;
    command.mission_point = mission_point;
    let commands = this.props.homeState.commands;
    this.props.setHomeState({
      commands: commands.concat(command),
      changedCommands: {startIndex: commands.length, endIndex: commands.length}
    });
  }

  //TODO make survey command work with protobuf format
  add_survey_command(boundary_pts, name, polygon) {
    let commands = this.props.homeState.commands.slice();

    commands.push({
      name: name || '',
      polygon: polygon,
      options: {
        command_type: 'survey',
        boundary_pts: boundary_pts,
        line_sep: 10,
        alt: 30
      }
    });

    this.props.setHomeState({
      commands: commands,
      changedCommands: {startIndex: commands.length, endIndex: commands.length}
    });
  }

  update_commands(commands) {
    for (let i = 0; i < commands.length; i++) {
      let command = commands[i];

      if (
        command.mission_point &&
        command.mission_point.marker &&
        command.mission_point.marker.getMap()
      ) {
        this.update_mission_point(command, i);
      } else if (this.commands[i]) {
        let type = command.type;
        let fields = command[type];

        this.commands[i].infowindow.setContent(this.command_info(i, type, fields));
        this.commands[i].onInfoChanged();
      }
    }
  }

  command_info(i, type, fields) {
    let title = i + 1 + ') ' + type;

    let info = '<div id="command_infowindow_' +
        i +
        '">' +
        '<h6>' +
        title +
        '</h6>' +
        'Lat: ' +
        fields.latitude +
        '<br>' +
        'Lng: ' +
        fields.longitude +
        '<br>';
    if (fields.altitude != undefined) {
      info += 'Alt: ' +
        fields.altitude +
        ' m<br>';
    }
    info += '<button class="remove_command btn btn-sm btn-outline-danger">' +
        'Remove</button>' +
        '<button class="drag_command btn btn-sm btn-outline-secondary">' +
        'Drag</button>' +
        '<button hidden class="place_command btn btn-sm btn-outline-success">' +
        'Place</button></div>'
    return info;
  }

  mission_point_title(mission_point_key, pos) {
    switch (mission_point_key) {
      case 'waypoints':
        return 'Waypoint ' + pos.order;
      case 'person':
        return 'Person';
      case 'off_axis_object':
        return 'Off-Axis Object';
      case 'home':
        return 'Home';
      case 'air_drop':
        return 'Air Drop';
      default:
        return null;
    }
  }

  mission_point_label(mission_point_key) {
    switch (mission_point_key) {
      case 'waypoints':
        return {
          fontFamily: 'Fontawesome',
          text: '\uf192',
          fontSize: '15px'
        };
      case 'person':
        return {
          fontFamily: 'Fontawesome',
          text: '\uf183',
          fontSize: '20px'
        };
      case 'off_axis_object':
        return {
          fontFamily: 'Fontawesome',
          text: '\uf030',
          fontSize: '14px'
        };
      case 'home':
        return {
          fontFamily: 'Fontawesome',
          text: '\uf015',
          fontSize: '18px'
        };
      case 'air_drop':
        return {
          fontFamily: 'Fontawesome',
          text: '\uf1e2',
          fontSize: '15px'
        };
      default:
        return null;
    }
  }

  draw_boundaries(fly_zones) {
    for (let polygon of this.fly_zones) {
      polygon.setMap(null);
    }

    this.fly_zones.length = 0;

    for (let fly_zone of fly_zones) {
      let bigRect = [];
      bigRect.push({
        lat: fly_zone.boundary_pts[0].latitude + 0.1,
        lng: fly_zone.boundary_pts[0].longitude - 0.1
      });
      bigRect.push({
        lat: fly_zone.boundary_pts[0].latitude - 0.1,
        lng: fly_zone.boundary_pts[0].longitude - 0.1
      });
      bigRect.push({
        lat: fly_zone.boundary_pts[0].latitude - 0.1,
        lng: fly_zone.boundary_pts[0].longitude + 0.1
      });
      bigRect.push({
        lat: fly_zone.boundary_pts[0].latitude + 0.1,
        lng: fly_zone.boundary_pts[0].longitude + 0.1
      });

      let boundary_coordinates = [];

      fly_zone.boundary_pts.sort((a, b) => a.order - b.order);

      for (let pt of fly_zone.boundary_pts) {
        boundary_coordinates.push({ lat: pt.latitude, lng: pt.longitude });
      }

      let paths = [boundary_coordinates];
      if (this.polygon_is_clockwise(boundary_coordinates)) {
        paths.push(bigRect);
      } else {
        paths.push(bigRect.slice().reverse());
      }

      let polygon = new google.maps.Polygon({
        paths: paths,
        strokeColor: '#FF0000',
        strokeOpacity: 0.4,
        strokeWeight: 2,
        fillColor: '#FF0000',
        fillOpacity: 0.2,
        clickable: false
      });

      polygon.setMap(this.map);
      this.fly_zones.push(polygon);
    }
  }

  draw_mission_waypoints(waypoints) {
    if (this.waypoint_path) {
      this.waypoint_path.setMap(null);
    }
    for (let waypoint of this.mission_points.waypoints) {
      waypoint.marker.setMap(null);
    }
    this.mission_points.waypoints.length = 0;

    if (waypoints === undefined) return;

    let positions = [];

    for (let pos of waypoints) {
      let mission_point = this.make_mission_marker('waypoints', pos);
      this.mission_points.waypoints.push(mission_point);
      positions.push({
        lat: pos.latitude,
        lng: pos.longitude
      });
    }

    let polyline = new google.maps.Polyline({
      path: positions,
      geodesic: true,
      strokeColor: '#00FFFF',
      strokeOpacity: 0,
      strokeWeight: 3,
      icons: [
        {
          icon: {
            path: 'M 0,-1 0,1',
            strokeOpacity: 0.7,
            scale: 4
          },
          offset: '0',
          repeat: '20px'
        }
      ]
    });

    polyline.setMap(this.map);
    this.waypoint_path = polyline;
  }

  draw_mission_point(mission_point_key, pos) {
    if (this.mission_points[mission_point_key]) {
      this.mission_points[mission_point_key].marker.setMap(null);
    }

    let mission_point = this.make_mission_marker(mission_point_key, pos);
    this.mission_points[mission_point_key] = mission_point;
  }

  update_mission_point(command, index) {
    let div = document.createElement('div');
    div.innerHTML = command.mission_point.infowindow.getContent();
    let command_info = index + 1 + ') ' + command.type + ': ';
    div.getElementsByClassName('command_info')[0].textContent = command_info;
    command.mission_point.infowindow.setContent(div.innerHTML);
    command.mission_point.onInfoChanged();
  }

  draw_target_search_area(points) {
    if (this.search_grid) {
      this.search_grid.setMap(null);
    }

    let boundary_coordinates = [];
    let avg_lat = 0;
    let avg_lng = 0;

    points.sort((a, b) => a.order - b.order);

    for (let pt of points) {
      boundary_coordinates.push({ lat: pt.latitude, lng: pt.longitude });
      avg_lat += pt.latitude;
      avg_lng += pt.longitude;
    }
    avg_lat /= boundary_coordinates.length;
    avg_lng /= boundary_coordinates.length;

    let polygon = new google.maps.Polygon({
      path: boundary_coordinates,
      strokeColor: '#00FF00',
      strokeOpacity: 0.4,
      strokeWeight: 1,
      fillColor: '#00FF00',
      fillOpacity: 0.2,
      clickable: true
    });

    let info = (
      <div id="mission_point_infowindow_search">
        <h6 class="infowindow_title">
          <span class="command_info" />
          Search Area
        </h6>
        <button class="add_point_to_plan btn btn-sm btn-outline-success">
          Add to Plan
        </button>
        <button
          hidden
          class="remove_point_from_plan btn btn-sm btn-outline-danger"
        >
          Remove from Plan
        </button>
      </div>
    );

    let infowindow = new google.maps.InfoWindow({
      content: ReactDOMServer.renderToString(info)
    });

    let setup_listeners = () => {
      let div = document.getElementById('mission_point_infowindow_search');
      if (div) {
        let add_btn = div.getElementsByClassName('add_point_to_plan')[0];
        let remove_btn = div.getElementsByClassName(
          'remove_point_from_plan'
        )[0];
        add_btn.onclick = () => {
          add_btn.setAttribute('hidden', 'hidden');
          remove_btn.removeAttribute('hidden');
          infowindow.setContent(div.outerHTML);
          setup_listeners();
          this.add_survey_command(
            boundary_coordinates,
            'Search Area',
            polygon
          );
        };
        remove_btn.onclick = () => {
          let command_index = this.props.homeState.commands.findIndex(
            el => el.polygon === polygon
          );
          if (command_index !== -1) {
            remove_btn.setAttribute('hidden', 'hidden');
            add_btn.removeAttribute('hidden');
            let command_info = div.getElementsByClassName('command_info')[0];
            command_info.textContent = '';
            infowindow.setContent(div.outerHTML);
            setup_listeners();
            let commands = this.props.homeState.commands.slice();
            commands.splice(command_index, 1);
            this.props.setHomeState({
              commands: commands,
              changedCommands: {startIndex: command_index, endIndex: commands.length}
            });
          }
        };
      }
    };

    google.maps.event.addListener(polygon, 'click', () => {
      infowindow.setPosition({ lat: avg_lat, lng: avg_lng });
      infowindow.open(this.map);
      setup_listeners();
    });

    google.maps.event.addListener(this.map, 'click', () => {
      infowindow.close();
    });

    polygon.setMap(this.map);
    this.search_grid = polygon;
  }

  make_mission_marker(mission_point_key, pos) {
    let coords = { lat: pos.latitude, lng: pos.longitude };
    let title = this.mission_point_title(mission_point_key, pos);
    let label = this.mission_point_label(mission_point_key);

    let marker_options = {
      map: this.map,
      position: coords,
      path: google.maps.SymbolPath.circle,
      strokeColor: '#FFFFFF',
      strokeOpacity: 0.8,
      strokeWeight: 3
    };
    if (label) {
      marker_options.label = label;
    }

    let marker = new google.maps.Marker(marker_options);

    let id = mission_point_key;
    if (pos.order) {
      id = id + '_' + pos.order;
    }

    let info = (
      <div id={'mission_point_infowindow_' + id}>
        <h6 class="infowindow_title">
          <span class="command_info" />
          {title}
        </h6>
        Lat: {coords.lat}
        <br />
        Lng: {coords.lng}
        {pos.altitude_msl ? (
          <span>
            <br />Alt: {pos.altitude_msl * METERS_PER_FOOT} m
          </span>
        ) : (
          <span />
        )}
        {mission_point_key !== 'off_axis_object' ? (
          <span>
            <br />
            <button class="add_point_to_plan btn btn-sm btn-outline-success">
              Add to Plan
            </button>
            <button
              hidden
              class="remove_point_from_plan btn btn-sm btn-outline-danger"
            >
              Remove from Plan
            </button>
          </span>
        ) : (
          <span />
        )}
      </div>
    );

    let infowindow = new google.maps.InfoWindow({
      content: ReactDOMServer.renderToString(info)
    });

    let mission_point = {
      marker: marker,
      infowindow: infowindow
    };

    let setup_listeners = () => {};
    if (mission_point_key !== 'off_axis_object') {
      setup_listeners = () => {
        let div = document.getElementById('mission_point_infowindow_' + id);
        if (div) {
          let add_btn = div.getElementsByClassName('add_point_to_plan')[0];
          let remove_btn = div.getElementsByClassName(
            'remove_point_from_plan'
          )[0];
          add_btn.onclick = () => {
            add_btn.setAttribute('hidden', 'hidden');
            remove_btn.removeAttribute('hidden');
            infowindow.setContent(div.outerHTML);
            mission_point.onInfoChanged();
            this.add_goto_command(
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
              remove_btn.setAttribute('hidden', 'hidden');
              add_btn.removeAttribute('hidden');
              let command_info = div.getElementsByClassName('command_info')[0];
              command_info.textContent = '';
              infowindow.setContent(div.outerHTML);
              mission_point.onInfoChanged();
              let commands = this.props.homeState.commands.slice();
              commands.splice(command_index, 1);
              this.props.setHomeState({
                commands: commands,
                changedCommands: {startIndex: command_index, endIndex: commands.length}
              });
            }
          };
        }
      };
    }

    mission_point.onInfoChanged = setup_listeners;
    mission_point.onInfoOpened = setup_listeners;

    marker.addListener('click', () => {
      infowindow.open(this.map, marker);
      mission_point.onInfoOpened();
    });

    google.maps.event.addListener(this.map, 'click', () => {
      infowindow.close();
    });

    return mission_point;
  }

  // Map camera control ########################################################
  pan_to_drone_if_following = props => {
    if (props.appState.followDrone) {
      this.map.setZoom(16);
      this.pan_to_drone();
    }
  };

  pan_to_drone() {
    this.map.panTo(this.drone_marker.getPosition());
  }

  // Obstacles #################################################################
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
    let pos = { lat: obstacle.latitude, lng: obstacle.longitude };
    let radius_feet = obstacle.cylinder_radius || obstacle.sphere_radius;
    let radius = radius_feet * METERS_PER_FOOT;

    let circle = new google.maps.Circle({
      center: pos,
      map: this.map,
      fillColor: '#FF0000',
      fillOpacity: 0.7,
      strokeWeight: 0,
      radius: radius,
      zIndex: 3
    });

    let infowindow = new google.maps.InfoWindow({
      content: this.make_obstacle_info(obstacle)
    });

    google.maps.event.addListener(circle, 'click', () => {
      infowindow.setPosition(circle.getCenter());
      infowindow.open(this.map);
    });

    google.maps.event.addListener(this.map, 'click', () => {
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

    let info = (
      <div>
        <h6>{obstacle.cylinder_radius ? 'Stationary' : 'Moving'} Obstacle</h6>
        Lat: {obstacle.latitude}
        <br />
        Lng: {obstacle.longitude}
        <br />
        Radius: {radius}m
        {obstacle.cylinder_height ? (
          <span>
            <br />Height: {obstacle.cylinder_height * METERS_PER_FOOT} m
          </span>
        ) : (
          <span />
        )}
        {obstacle.altitude_msl ? (
          <span>
            <br />Alt: {obstacle.altitude_msl * METERS_PER_FOOT} m
          </span>
        ) : (
          <span />
        )}
      </div>
    );

    return ReactDOMServer.renderToString(info);
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

  // Math helper functions #####################################################
  rad(x) {
    return x * Math.PI / 180;
  }

  polygon_is_clockwise(vertices) {
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
