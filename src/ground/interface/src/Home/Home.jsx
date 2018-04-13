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
    focusedCommand: null
  };

  command_types = {};

  commandChangeSendQueue = [];
  lastCommandSendTime = 0;

  componentDidMount() {
    protobuf.load('mission_commands.proto', (err, root) => {
      if (err) throw err;

      let proto_commands = root.toJSON().nested.lib.nested.mission_manager
        .nested;

      for (let proto_command in proto_commands) {
        if (proto_command == 'Mission' || proto_command == 'Command') break;

        let fields = Array();
        for (let field in proto_commands[proto_command].fields) {
          let field_info = proto_commands[proto_command].fields[field];
          field_info.name = field;
          fields.push(field_info);
        }

        this.command_types[proto_command] = fields;
      }

      this.protobuf_root = root;


      this.props.socketOn('commands_changed', newState => {
        if (this.state.invalidCommands) {
          newState.changedCommands = null;
          newState.invalidCommands = false;
        } else if (newState.changedCommands) {
          for (let i = 0; i < newState.changedCommands.startIndex; i++) {
            newState.commands[i] = this.state.commands[i];
          }
          for (let i = newState.changedCommands.endIndex+1; 
               i < this.state.commands.length && i < newState.commands.length; i++) {
            newState.commands[i] = this.state.commands[i];
          }
        }
        newState.dontSendCommandChanges = true;
        this.setState(newState);
      });

      this.props.socketEmit('request_commands');
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

        <Controls homeState={this.state} socketEmit={this.props.socketEmit}
                  getMission={this.get_mission} />
      </div>
    );
  }

  componentWillReceiveProps(nextProps) {
    if (nextProps.appState.missions !== this.props.appState.missions) {
      if (nextProps.appState.missions.length > 0) {
        this.setState({ mission: nextProps.appState.missions[0] });
      } else {
        this.setState({ mission: {} });
      }
    }
  }

  setHomeState = newState => {
    if (newState.commands && this.state.invalidCommands && newState.invalidCommands !== false) {
      return false;
    }

    this.setState(newState);

    if (newState.commands && !newState.dontSendCommandChanges) {
      let commands_info = [];
      for (let command of newState.commands) {
        let command_info = {};
        command_info.type = command.type;
        command_info[command.type] = command[command.type];
        command_info.interop_object = command.interop_object;
        command_info.name = command.name;
        commands_info.push(command_info);
      }
      let obj = {
        commands: commands_info,
        changedCommands: newState.changedCommands
      };

      let currentTime = Date.now();
      let timePassed = currentTime - this.lastCommandSendTime;
      if (timePassed >= 250 && this.commandChangeSendQueue.length === 0) {
        this.lastCommandSendTime = currentTime;
        this.props.socketEmit('commands_changed', obj);
      } else {
        if (this.commandChangeSendQueue.length === 0) {
          setTimeout(this.sendQueuedCommands, 250-timePassed);
        }
        this.addToCommandSendQueue(obj);
      }
    }

    return true;
  };

  sendQueuedCommands = () => {
    this.lastCommandSendTime = Date.now();
    let obj = this.commandChangeSendQueue.shift();
    this.props.socketEmit('commands_changed', obj);
    if (this.commandChangeSendQueue.length > 0) {
      setTimeout(this.sendQueuedCommands, 250);
    }
  }

  addToCommandSendQueue(obj) {
    let lastChange = this.commandChangeSendQueue[this.commandChangeSendQueue.length-1];
    lastChange = lastChange ? lastChange.changedCommands : null;
    if (lastChange && obj.changedCommands &&
        obj.changedCommands.startIndex <= lastChange.endIndex+1 &&
        obj.changedCommands.endIndex >= lastChange.startIndex-1) {
      obj.changedCommands.startIndex = Math.min(
        obj.changedCommands.startIndex,
        lastChange.startIndex
      );
      obj.changedCommands.endIndex = Math.max(
        obj.changedCommands.endIndex,
        lastChange.endIndex
      );
      this.commandChangeSendQueue[this.commandChangeSendQueue.length-1] = obj;
    } else {
      this.commandChangeSendQueue.push(obj);
    }
  }

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

  make_inner_command = (type, fields) => {
    let defaults = {
      // 'GotoCommand': {
      //   latitude: 38.145298,
      //   longitude: -76.42861,
      //   altitude: 30
      // },
      // 'BombCommand': {
      //   latitude: 38.145298,
      //   longitude: -76.42861
      // }
    };

    let command_proto_defs = Object();
    for (let name in this.command_types) {
      let cmd = this.protobuf_root.lookupType('lib.mission_manager.' + name);

      command_proto_defs[name] = cmd;
    }

    if (fields == null) {
      if (defaults[type]) {
        fields = defaults[type];
      } else {
        fields = {};

        for (let field of this.command_types[type]) {
          fields[field.name] = this.add_field(type, field);
        }
      }
    } else {
      // delete extra fields
      for(let field in fields) {
        if (!this.command_types[type].find(el => el.name === field)) {
          delete fields[field];
        }
      }

      // add missing fields
      for(let field of this.command_types[type]) {
        if (fields[field.name] == undefined) {
          fields[field.name] = defaults[type] ? defaults[type][field] || 
            this.add_field(type, field) : this.add_field(type, field);
        }
      }
    }

    let cmd_inner = command_proto_defs[type].create(fields);
    return cmd_inner;
  };

  make_command = (type, fields) => {
    let Command = this.protobuf_root.lookupType('lib.mission_manager.Command');

    let cmd_inner = this.make_inner_command(type, fields);

    let cmd_oneof = {};
    cmd_oneof[type] = cmd_inner;

    let cmd = Command.create(cmd_oneof);
    cmd.type = type;

    return cmd;
  };

  add_field(command_type, field) {
    if (field.type === 'Mission') {
      return null;
    }
    if (Object.keys(this.command_types).includes(field.type)) {
      if (this.command_types[command_type].find(el => el.name === field.name).rule === 'repeated') {
        return [this.make_inner_command(field.type, null), this.make_inner_command(field.type, null)];
      }
      return this.make_inner_command(field.type, null);
    }
    return 0;
  }
}

export default Home;
