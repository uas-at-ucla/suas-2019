import React, { Component } from 'react';
import Navbar from './Navbar/Navbar'
import './App.css';
import io from 'socket.io-client/dist/socket.io.js';

import Home from './Home/Home';
import Analytics from './Analytics/Analytics';

class App extends Component {
  constructor(props) {
    super(props);

    this.state = {
      optionSelected: 'Home', // Default is Home
      droneArmedStatus: "Offline",
      droneState: "",
      telemetryText: {},
      interopBtnText: "Connect to Interop",
      interopBtnEnabled: true
    };

    this.handleTab = this.handleTab.bind(this);
    this.interopBtnClick = this.interopBtnClick.bind(this);
  }

  // Render based on button clicks in Navbar
  renderSelection() {
    if (this.state.optionSelected === "Home") {
      return (
        <Home ref="home" telemetryText={this.state.telemetryText}
              droneState={this.state.droneState}
              droneArmedStatus={this.state.droneArmedStatus}/>
      );
    }
    else if (this.state.optionSelected === "Analytics") {
      return (
        <Analytics/>
      );
    }
  }

  handleTab(option) {
    this.setState({
      optionSelected: option,
    });
  }

  render() {
    return (
<div className="App">
  <Navbar interopBtnClick={this.interopBtnClick}
          interopBtnText={this.state.interopBtnText}
          interopBtnEnabled={this.state.interopBtnEnabled}
          handleTab={this.handleTab}/>
  {this.renderSelection()}
</div>
    );
  }

  componentDidMount() {
    this.map = this.refs.home.refs.mapScreen.refs.map;
    this.map_screen = this.refs.home.refs.mapScreen;

    var SOCKET_DOMAIN = "0.0.0.0";
    var SOCKET_PORT = 8084;

    var SOCKET_ADDRESS = "http://" + SOCKET_DOMAIN + ":" + SOCKET_PORT;

    this.socket = io.connect(SOCKET_ADDRESS);

    this.socket.on('connect', () => {
      console.log("Connected to ground interface feeder!");
      this.setState({droneArmedStatus: "Online"});
      this.setState({droneState: ""});
    });

    this.socket.on('disconnect', () => {
      console.log("Disconnected from ground interface feeder!");
      this.setState({droneArmedStatus: "Offline"});
      this.setState({droneState: ""});
    });

    this.socket.on('telemetry', (telemetry) => {
      console.log("got something!");
      this.map.update_drone_position(
        Number(telemetry["gps_lat"]),
        Number(telemetry["gps_lng"]),
        Number(telemetry["heading"])
      );

      let armed = telemetry["armed"] === "True" ? "Armed" : "Disarmed";

      this.map_screen.setState({
        fullState: {
          armed: armed,
          state: this.convert_to_title_text(telemetry["state"])
        }
      });

      console.log(telemetry);

      var METERS_PER_SECOND_TO_MPH = 2.23694;
      this.map_screen.setState({
        telemetryText: {
          speed: "" + this.round(telemetry["air_speed"] * METERS_PER_SECOND_TO_MPH, 1)
            + "mph",
          altitude: "" + this.round(telemetry["gps_rel_alt"], 1) + " meters",
          position: "" + this.round(telemetry["gps_lat"], 7) + ", "
            + this.round(telemetry["gps_lng"], 7),
          satellites: "" + this.round(telemetry["gps_satellites"], 7),
          heading: "" + this.round(telemetry["heading"], 7)
        }
      })
    });

    this.socket.on('initial_data', (data) => {
      this.received_interop_status(data.interop_connected);
      if (data.interop_connected) {
        this.map.set_stationary_obstacles(data.stationary_obstacles);
        this.map.set_moving_obstacles(data.moving_obstacles);
        var mission = data.missions[0];
        for (var fly_zone of mission.fly_zones) {
          this.map.draw_boundary(fly_zone.boundary_pts);
        }
      }
    });

    this.socket.on('moving_obstacles', (moving_obstacles) => {
      if (!this.map.update_moving_obstacles(moving_obstacles)) {
        console.log("Different moving obstacles received.");
        this.map.set_moving_obstacles(moving_obstacles);
      }
    });

    this.socket.on('interop_connected', (is_interop_connected) =>
      this.received_interop_status(is_interop_connected)
    );
  }

  received_interop_status(is_interop_connected) {
    if (is_interop_connected) {
      this.setState({interopBtnText: "Connected to Interop"});
      this.setState({interopBtnEnabled: false});
    } else {
      this.setState({interopBtnText: "Cannot Connect to Interop Server!"});
      this.setState({interopBtnEnabled: true});
      setTimeout(() => this.setState({interopBtnText: "Connect to Interop"}), 1000);
    }
  }

  interopBtnClick() {
    this.setState({interopBtnText: "Connecting..."});
    this.setState({interopBtnEnabled: false});
    this.socket.emit('connect_to_interop');
  }

  convert_to_title_text(str) {
    return str.replace(/\w\S*/g, function(txt){
      return txt.charAt(0).toUpperCase() + txt.substr(1).toLowerCase();
    });
  }

  round(value, precision) {
    var multiplier = Math.pow(10, precision || 0);
    return Math.round(value * multiplier) / multiplier;
  }
}

export default App;
