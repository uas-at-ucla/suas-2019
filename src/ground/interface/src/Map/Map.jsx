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
      //lat: 38.145298,
      //lng: -76.42861
      lat: 34.175048,
      lng: -118.48159
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
        minZoom: 1
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

    this.drone_path = new google.maps.Polyline({
      map: this.map,
      path: [],
      geodesic: true,
      strokeColor: '#F76926',
      strokeOpacity: 0.7,
      strokeWeight: 2,
      zIndex: 100
    });
    this.drone_path.setMap(this.map);

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

    // Close all infowindows on map click
    this.map.addListener('click', () => {
      for (let command of this.commands) {
        if (command) {
          command.infowindow.close();
        }
      }
      for (let key in this.mission_points) {
        if (Array.isArray(this.mission_points[key])) {
          for (let mission_point of this.mission_points[key]) {
            mission_point.infowindow.close();
          }
        } else if (this.mission_points[key]) {
          this.mission_points[key].infowindow.close();
        }
      }
      if (this.search_grid) {
        this.search_grid.infowindow.close();
      }
      for (let obstacle of this.stationary_obstacles) {
        obstacle.infowindow.close();
      }
      for (let obstacle of this.moving_obstacles) {
        obstacle.infowindow.close();
      }
    });

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
      'droneCommands',
      this.draw_drone_command_path
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

  get_command_pos(command, type) {
    let key = this.props.getCommandPosKey(type);
    if (key) {
      return command[key];
    }

    return null;
  }

  draw_command_path = props => {
    let commands = props.homeState.commands;

    let startIndex = 0;
    let endIndex = Math.max(commands.length, this.commands.length) - 1;
    if (props.homeState.changedCommands !== null) {
      startIndex = props.homeState.changedCommands.startIndex;
      endIndex = props.homeState.changedCommands.endIndex;
    }

    // Reset command annotations on map.
    for (let i = startIndex; i <= endIndex; i++) {
      if (this.commands[i]) {
        this.commands[i].marker.setMap(null);
      } else if (this.props.homeState.commands[i]) {
        for (let geometry of ['mission_point', 'mission_polygon']) {
          if (this.props.homeState.commands[i][geometry]) {
            let command_index = commands.findIndex(
              el => el[geometry] === this.props.homeState.commands[i][geometry]
            );
            if (command_index === -1) {
              this.update_mission_point(
                this.props.homeState.commands[i],
                i,
                false,
                geometry
              );
            }
          }
        }
      }
      if (this.commands_path[i]) {
        this.commands_path[i].setMap(null);
      }
    }
    for (let i = endIndex + 1; i < this.commands_path.length; i++) {
      if (this.commands_path[i]) {
        this.commands_path[i].setMap(null);
        break;
      }
    }

    this.commands.length = commands.length;
    this.commands_path.length = commands.length;

    // Draw the command path.
    const line_symbol = {
      path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
      strokeColor: '#0000FF'
    };

    let last_pos = null;

    for (let i = 0; i <= startIndex - 1; i++) {
      let command = commands[i];
      let type = command.type;
      let fields = command[type];
      let pos = this.get_command_pos(fields, type);
      if (pos) {
        last_pos = { lat: pos.latitude, lng: pos.longitude };
      } else if (type === 'SurveyCommand') {
        last_pos = this.get_midpoint(command.SurveyCommand.surveyPolygon);
      }
    }

    let new_commands = commands.slice();
    let new_start_index = -1;
    let new_end_index = -1;

    for (let i = startIndex; i < commands.length; i++) {
      let command = commands[i];
      let type = command.type;
      let fields = command[type];
      let pos = this.get_command_pos(fields, type);

      if (i <= endIndex) {
        if (command.interop_object) {
          if (
            command.mission_point &&
            command.mission_point.marker.getMap() != null
          ) {
            this.update_mission_point(command, i, true, 'mission_point');
            this.commands[i] = null;
          } else if (
            command.mission_polygon &&
            command.mission_polygon.polygon.getMap() != null
          ) {
            this.update_mission_point(command, i, true, 'mission_polygon');
            this.commands[i] = null;
          } else {
            let mission_point = null;
            let mission_polygon = null;
            if (!command.mission_point && !command.mission_polygon) {
              if (command.interop_object === 'search_area') {
                if (
                  this.search_grid &&
                  this.same_location(
                    this.search_grid.infowindow.getPosition(),
                    this.get_midpoint(command.SurveyCommand.surveyPolygon)
                  )
                ) {
                  mission_polygon = this.search_grid;
                }
              } else if (command.interop_object.split('_')[0] === 'waypoints') {
                let waypoint_index = command.interop_object.split('_')[1] - 1;
                let waypoint = this.mission_points['waypoints'][waypoint_index];
                if (
                  waypoint &&
                  this.same_location(pos, waypoint.marker.getPosition())
                ) {
                  mission_point = waypoint;
                }
              } else {
                let goal = this.mission_points[command.interop_object];
                if (
                  goal &&
                  this.same_location(pos, goal.marker.getPosition())
                ) {
                  mission_point = goal;
                }
              }
            }

            if (mission_point || mission_polygon) {
              new_commands[i].mission_point = mission_point;
              new_commands[i].mission_polygon = mission_polygon;
              if (new_start_index === -1) {
                new_start_index = i;
              }
              new_end_index = i;
            } else if (props.homeState.restoreCommands) {
              new_commands[i].mission_point = null;
              new_commands[i].mission_polygon = null;
              new_commands[i].name = null;
              new_commands[i].interop_object = null;
              if (new_start_index === -1) {
                new_start_index = i;
              }
              new_end_index = i;
            } else {
              let err_command = this.props.makeCommand('NothingCommand', null);
              err_command.name = 'Commands do not agree with interop data!!!';
              this.props.setHomeState({
                commands: [err_command],
                changedCommands: null,
                dontSendCommandChanges: true,
                invalidCommands: true
              });
              return;
            }
          }
        } else if (pos) {
          let marker = new google.maps.Marker({
            map: this.map,
            position: { lat: pos.latitude, lng: pos.longitude }
          });

          let infowindow = new google.maps.InfoWindow({
            content: this.command_info(i, type, pos)
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
                  changedCommands: { startIndex: i, endIndex: commands.length }
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
            pos.latitude = marker.getPosition().lat();
            pos.longitude = marker.getPosition().lng();

            this.props.setHomeState({
              commands: commands,
              changedCommands: { startIndex: i, endIndex: i }
            });
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

      if (pos || command.type === 'SurveyCommand') {
        let this_pos = null;
        if (pos) {
          this_pos = { lat: pos.latitude, lng: pos.longitude };
        } else if (command.type === 'SurveyCommand') {
          this_pos = this.get_midpoint(command.SurveyCommand.surveyPolygon);
        }
        if (last_pos != null) {
          // Create the polyline and add the symbol via the 'icons' property.
          this.commands_path[i] = new google.maps.Polyline({
            path: [last_pos, this_pos],
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

          if (i > endIndex) {
            break;
          }
        }
        last_pos = this_pos;
      } else {
        this.commands_path[i] = null;
      }
    }

    if (new_start_index !== -1) {
      let new_state = {
        commands: new_commands,
        changedCommands: {
          startIndex: new_start_index,
          endIndex: new_end_index
        },
        dontSendCommandChanges: true
      };
      if (props.homeState.restoreCommands) {
        new_state.dontSendCommandChanges = false;
        new_state.restoreCommands = false;
      }
      this.props.setHomeState(new_state);
    } else if (props.homeState.restoreCommands) {
      this.props.setHomeState({ restoreCommands: false });
    }
  };

  focus_on_command = props => {
    if (props.homeState.focusedCommand !== null) {
      this.props.setAppState({ followDrone: false });
      this.props.setHomeState({ focusedCommand: null });
      let command = props.homeState.commands[props.homeState.focusedCommand];
      let point =
        this.commands[props.homeState.focusedCommand] ||
        command.mission_point;
      let pos = null;
      let marker = null;
      if (point && point.marker) {
        marker = point.marker;
        pos = marker.getPosition();
      } else if (command.type === 'SurveyCommand') {
        pos = this.get_midpoint(command.SurveyCommand.surveyPolygon);
      }
      if (pos) {
        this.map.panTo(pos);
      }
      if (marker) {
        point.marker.setAnimation(google.maps.Animation.BOUNCE);
        setTimeout(() => point.marker.setAnimation(null), 1400);
      }
    }
  };

  draw_drone_command_path = props => {
    if (this.drone_command_path) {
      this.drone_command_path.setMap(null);
    }
    if (this.drone_current_command_path) {
      this.drone_current_command_path.setMap(null);
    }

    let points = [];
    let current_command_points = [];
    for (let i = 0; i < props.homeState.droneCommands.length; i++) {
      let command = props.homeState.droneCommands[i];
      if (command.subMission) {
        if (i === props.homeState.droneCurrentCommand && points.length > 0) {
          current_command_points.push(points[points.length - 1]);
        }
        for (let subCommand of command.subMission.commands) {
          if (subCommand.GotoRawCommand) {
            let point = {
              lat: subCommand.GotoRawCommand.goal.latitude,
              lng: subCommand.GotoRawCommand.goal.longitude
            };
            points.push(point);
            if (i === props.homeState.droneCurrentCommand) {
              current_command_points.push(point);
            }
          }
        }
      }
    }

    let polyline = new google.maps.Polyline({
      path: points,
      geodesic: true,
      strokeColor: '#FFD700',
      strokeOpacity: 1,
      strokeWeight: 2,
      zIndex: 11
    });
    polyline.setMap(this.map);
    this.drone_command_path = polyline;

    polyline = new google.maps.Polyline({
      path: current_command_points,
      geodesic: true,
      strokeColor: '#00FF00',
      strokeOpacity: 1,
      strokeWeight: 2,
      zIndex: 12
    });
    polyline.setMap(this.map);
    this.drone_current_command_path = polyline;
  };

  update_drone_position = props => {
    let telemetry = props.appState.telemetry;

    if (!telemetry) return;

    let new_position = new google.maps.LatLng(
      telemetry.sensors.latitude,
      telemetry.sensors.longitude
    );

    let current_path = this.drone_path.getPath();
    if (
      current_path.getLength() == 0 ||
      this.get_distance(
        new_position,
        current_path.getAt(current_path.getLength() - 1)
      ) > 10.0
    ) {
      this.drone_path.getPath().push(new_position);
    }
    if (current_path.getLength() > 50) {
      this.drone_path.getPath().removeAt(0);
    }

    this.drone_marker.setPosition(new_position);
    this.drone_marker_icon.rotation = telemetry.sensors.heading;
    this.drone_marker.setIcon(this.drone_marker_icon);

    if (this.get_distance(new_position, this.map.getCenter()) > 50.0) {
      if (this.props.appState.followDrone) {
        this.pan_to_drone();
      }
    }
  };

  add_goto_command(lat, lng, alt, name, mission_point, interop_object_name) {
    let command = this.props.makeCommand('GotoCommand', {
      goal: {
        latitude: lat,
        longitude: lng,
        altitude: alt
      },
      comeToStop: true
    });
    command.name = name;
    command.mission_point = mission_point;
    command.interop_object = interop_object_name;
    let commands = this.props.homeState.commands;
    return this.props.setHomeState({
      commands: commands.concat(command),
      changedCommands: {
        startIndex: commands.length,
        endIndex: commands.length
      }
    });
  }

  add_survey_command(
    boundary_pts,
    alt,
    name,
    mission_polygon,
    interop_object_name
  ) {
    let fields = {
      surveyPolygon: [],
      altitude: alt
    };
    for (let pt of boundary_pts) {
      fields.surveyPolygon.push({
        latitude: pt.lat,
        longitude: pt.lng
      });
    }
    let command = this.props.makeCommand('SurveyCommand', fields);
    command.name = name;
    command.mission_polygon = mission_polygon;
    command.interop_object = interop_object_name;
    let commands = this.props.homeState.commands;
    return this.props.setHomeState({
      commands: commands.concat(command),
      changedCommands: {
        startIndex: commands.length,
        endIndex: commands.length
      }
    });
  }

  command_info(i, type, fields) {
    let title = i + 1 + ') ' + type;

    let info =
      '<div id="command_infowindow_' +
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
      info += 'Alt: ' + fields.altitude + ' m<br>';
    }
    info +=
      '<button class="remove_command btn btn-sm btn-outline-danger">' +
      'Remove</button>' +
      '<button class="drag_command btn btn-sm btn-outline-secondary">' +
      'Drag</button>' +
      '<button hidden class="place_command btn btn-sm btn-outline-success">' +
      'Place</button></div>';
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

    if (!fly_zones) return;

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

      let command_index = this.props.homeState.commands.findIndex(
        el => el.mission_point === waypoint
      );
      if (command_index !== -1) {
        this.props.setHomeState({
          commands: this.props.homeState.commands.slice(),
          changedCommands: {
            startIndex: command_index,
            endIndex: command_index
          }
        });
      }
    }
    this.mission_points.waypoints.length = 0;

    if (!waypoints) return;

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

      let command_index = this.props.homeState.commands.findIndex(
        el => el.mission_point === this.mission_points[mission_point_key]
      );
      if (command_index !== -1) {
        this.props.setHomeState({
          commands: this.props.homeState.commands.slice(),
          changedCommands: {
            startIndex: command_index,
            endIndex: command_index
          }
        });
      }
    }

    if (!pos) return;

    let mission_point = this.make_mission_marker(mission_point_key, pos);
    this.mission_points[mission_point_key] = mission_point;
  }

  update_mission_point(command, index, in_mission, geometry) {
    let div = document.createElement('div');
    div.innerHTML = command[geometry].infowindow.getContent();
    let command_info_el = div.getElementsByClassName('command_info')[0];
    let command_alt_el = div.getElementsByClassName('command_alt')[0];
    let add_btn = div.getElementsByClassName('add_point_to_plan')[0];
    let remove_btn = div.getElementsByClassName('remove_point_from_plan')[0];
    if (in_mission) {
      let type = command.type;
      let fields = command[type];
      let pos = this.get_command_pos(fields, type);
      if (pos && pos.altitude != null) {
        command_alt_el.textContent = 'Commanded Alt: ' + pos.altitude + ' m';
      }
      let command_info = index + 1 + ') ' + command.type + ': ';
      command_info_el.textContent = command_info;
      add_btn.setAttribute('hidden', 'hidden');
      remove_btn.removeAttribute('hidden');
    } else {
      command_alt_el.textContent = '';
      command_info_el.textContent = '';
      remove_btn.setAttribute('hidden', 'hidden');
      add_btn.removeAttribute('hidden');
    }
    command[geometry].infowindow.setContent(div.innerHTML);
    command[geometry].onInfoChanged();
  }

  draw_target_search_area(points) {
    if (this.search_grid) {
      this.search_grid.polygon.setMap(null);

      let command_index = this.props.homeState.commands.findIndex(
        el => el.mission_point === this.search_grid
      );
      if (command_index !== -1) {
        this.props.setHomeState({
          commands: this.props.homeState.commands.slice(),
          changedCommands: {
            startIndex: command_index,
            endIndex: command_index
          }
        });
      }
    }

    if (!points) return;

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
      <div id="infowindow_search_area">
        <h6 class="infowindow_title">
          <span class="command_info" />
          Search Area
        </h6>
        <span class="command_alt" />
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
      content: ReactDOMServer.renderToString(info),
      position: { lat: avg_lat, lng: avg_lng }
    });

    let mission_polygon = {
      polygon: polygon,
      infowindow: infowindow
    };

    let setup_listeners = () => {
      let div = document.getElementById('infowindow_search_area');
      if (div) {
        let add_btn = div.getElementsByClassName('add_point_to_plan')[0];
        let remove_btn = div.getElementsByClassName(
          'remove_point_from_plan'
        )[0];
        add_btn.onclick = () => {
          this.add_survey_command(
            boundary_coordinates,
            30,
            'Search Area',
            mission_polygon,
            'search_area'
          );
        };
        remove_btn.onclick = () => {
          let command_index = this.props.homeState.commands.findIndex(
            el => el.mission_polygon === mission_polygon
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
              changedCommands: {
                startIndex: command_index,
                endIndex: commands.length
              }
            });
          }
        };
      }
    };

    mission_polygon.onInfoChanged = setup_listeners;
    mission_polygon.onInfoOpened = setup_listeners;

    google.maps.event.addListener(polygon, 'click', () => {
      infowindow.open(this.map);
      mission_polygon.onInfoOpened();
    });

    polygon.setMap(this.map);
    this.search_grid = mission_polygon;
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

    if (pos.altitude_msl) coords.alt = pos.altitude_msl * METERS_PER_FOOT;

    let info = (
      <div id={'mission_point_infowindow_' + id}>
        <h6 class="infowindow_title">
          <span class="command_info" />
          {title}
        </h6>
        Lat: {coords.lat}
        <br />
        Lng: {coords.lng}
        {coords.alt ? (
          <span>
            <br />Alt: {coords.alt} m
          </span>
        ) : (
          <span />
        )}
        <br />
        <span class="command_alt" />
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
            this.add_goto_command(
              coords.lat,
              coords.lng,
              coords.alt || 30,
              title,
              mission_point,
              id
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
                changedCommands: {
                  startIndex: command_index,
                  endIndex: commands.length
                }
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

  get_midpoint(pts) {
    let avg_lat = 0;
    let avg_lng = 0;
    let num = 0;
    for (let pt of pts) {
      avg_lat += pt.latitude;
      avg_lng += pt.longitude;
      num++;
    }
    avg_lat /= num;
    avg_lng /= num;
    return { lat: avg_lat, lng: avg_lng };
  }

  same_location(pt1, pt2) {
    let lat1 =
      typeof pt1.lat == 'function' ? pt1.lat() : pt1.lat || pt1.latitude;
    let lng1 =
      typeof pt1.lng == 'function' ? pt1.lng() : pt1.lng || pt1.longitude;
    let lat2 =
      typeof pt2.lat == 'function' ? pt2.lat() : pt2.lat || pt2.latitude;
    let lng2 =
      typeof pt2.lng == 'function' ? pt2.lng() : pt2.lng || pt2.longitude;
    return Math.abs(lat1 - lat2) < 0.000001 && Math.abs(lng1 - lng2) < 0.000001;
  }
}

export default Map;
