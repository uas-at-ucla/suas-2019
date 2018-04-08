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
    command_types: {},
    commands: [],
    dontRedrawCommands: false,
    focusedCommand: null,
    add_command: null,
    make_command: null,
    trigger_redraw: null,
    set_home_state: null
  };

  componentDidMount() {
    let self = this;

    self.state.set_home_state = self.setHomeState;

    protobuf.load('mission_commands.proto', function(err, root) {
      if (err) throw err;

      let proto_commands = root.toJSON().nested.lib.nested.mission_manager
        .nested;
      self.state.add_command = self.add_command;
      self.state.make_command = self.make_command;
      self.state.trigger_redraw = self.trigger_redraw;

      for (let proto_command in proto_commands) {
        if (proto_command == 'Mission' || proto_command == 'Command') break;

        let fields = Array();
        for (let field in proto_commands[proto_command].fields) {
          fields.push(field);
        }

        self.state.command_types[proto_command] = fields;
      }

      self.protobuf_root = root;
    });
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
                ref={this.sidebar}
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

  make_command = (type, fields) => {
    let self = this;
    let Mission = self.protobuf_root.lookupType('lib.mission_manager.Mission');
    let Command = self.protobuf_root.lookupType('lib.mission_manager.Command');

    let command_proto_defs = Object();
    for (let name of Object.keys(self.state.command_types)) {
      let cmd = self.protobuf_root.lookupType('lib.mission_manager.' + name);

      command_proto_defs[name] = cmd;
    }

    if (fields == null) {
      if (type == 'GotoCommand') {
        fields = {
          latitude: 38.145298,
          longitude: -76.42861,
          altitude: 10
        };
      } else {
        fields = {};

        for(let field of self.state.command_types[type]) {
          console.log(field);
          fields[field] = 0;
        }
      }
    }

    console.log(fields);

    let cmd_inner = command_proto_defs[type].create(fields);

    let cmd_oneof = {};
    cmd_oneof[type] = cmd_inner;

    let cmd = Command.create(cmd_oneof);

    return cmd;
  };

  add_command = (type, fields) => {
    let self = this;

    let cmds = self.state.commands.slice();
    cmds.push(self.make_command(type, fields));

    self.setState({commands: cmds});
  };

  trigger_redraw = () => {
    this.setState({});
  }
}

export default Home;
