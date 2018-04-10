import React, { Component } from 'react';

import './Home.css';
import Sidebar from '../Sidebar/Sidebar';
import Map from '../Map/Map';
import Telemetry from '../Telemetry/Telemetry';
import Controls from '../Controls/Controls';

let protobuf = require('protobufjs');

class Home extends Component {
  state = {
    isSidebarShown: true,
    mission: this.props.appState.missions[0] || null,
    commands: [],
    changedCommands: null,
    focusedCommand: null,
    get_mission: null
  };

  command_types = {};

  componentDidMount() {
    protobuf.load('mission_commands.proto', (err, root) => {
      if (err) throw err;

      let proto_commands = root.toJSON().nested.lib.nested.mission_manager
        .nested;

      for (let proto_command in proto_commands) {
        if (proto_command == 'Mission' || proto_command == 'Command') break;

        let fields = Array();
        for (let field in proto_commands[proto_command].fields) {
          fields.push(field);
        }

        this.command_types[proto_command] = fields;
      }

      this.protobuf_root = root;
    });

    this.state.get_mission = this.get_mission;
  }

  render() {
    return (
      <div className="Home">
        <div id="sides">
          <Map
            id="map"
            appState={this.props.appState}
            setAppState={this.props.setAppState}
            homeState={this.state}
            setHomeState={this.setHomeState}
            makeCommand={this.make_command}
          />
          <div id="left_side">
            <div
              id="sidebar_container"
              className={!this.state.isSidebarShown ? 'hidden' : null}
            >
              <Sidebar
                appState={this.props.appState}
                homeState={this.state}
                setHomeState={this.setHomeState}
                socketEmit={this.props.socketEmit}
                makeCommand={this.make_command}
                commandTypes={this.command_types}
              />
            </div>
          </div>
          <div id="right_side">
            <Telemetry appState={this.props.appState} homeState={this.state} />
          </div>
        </div>

        <Controls homeState={this.state} socketEmit={this.props.socketEmit} />
      </div>
    );
  }

  componentWillReceiveProps(nextProps) {
    if (
      nextProps.appState.missions.length > 0 &&
      nextProps.appState.missions !== this.props.appState.missions
    ) {
      this.setState({ mission: nextProps.appState.missions[0] });
    }
  }

  setHomeState = newState => {
    this.setState(newState);
  };

  toggleSidebar = () => {
    this.setState({ isSidebarShown: !this.state.isSidebarShown });
  };

  get_mission = () => {
    const Mission = this.protobuf_root.lookupType('lib.mission_manager.Mission');

    const cmds = {commands: this.state.commands.slice()};
    const mission = Mission.create(cmds);
    const serialized_mission = Mission.encode(mission).finish();

    return serialized_mission;
  }

  make_command = (type, fields) => {
    let Command = this.protobuf_root.lookupType('lib.mission_manager.Command');

    let defaults = {
      'GotoCommand': {
        latitude: 38.145298,
        longitude: -76.42861,
        altitude: 30
      },
      'BombCommand': {
        latitude: 38.145298,
        longitude: -76.42861
      }
    };

    let command_proto_defs = Object();
    for (let name of Object.keys(this.command_types)) {
      let cmd = this.protobuf_root.lookupType('lib.mission_manager.' + name);

      command_proto_defs[name] = cmd;
    }

    if (fields == null) {
      if (defaults[type]) {
        fields = defaults[type];
      } else {
        fields = {};

        for(let field of this.command_types[type]) {
          fields[field] = 0;
        }
      }
    } else {
      // delete extra fields
      for(let field of Object.keys(fields)) {
        if (!this.command_types[type].includes(field)) {
          delete fields[field];
        }
      }

      // add missing fields
      for(let field of this.command_types[type]) {
        if (fields[field] == undefined) {
          fields[field] = defaults[type] ? defaults[type][field] || 0 : 0;
        }
      }
    }

    let cmd_inner = command_proto_defs[type].create(fields);

    let cmd_oneof = {};
    cmd_oneof[type] = cmd_inner;

    let cmd = Command.create(cmd_oneof);
    cmd.type = type;

    return cmd;
  };
}

export default Home;
