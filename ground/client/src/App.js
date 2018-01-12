import React, { Component } from 'react';
import Navbar from './Navbar/Navbar'
import './App.css';
import io from 'socket.io-client/dist/socket.io.js';

import Home from './Home/Home';
import Analytics from './Analytics/Analytics';
import Images from './Images/Images.js';

class App extends Component {
  constructor(props) {
    super(props);
    this.state = {
      optionSelected: 'Home', // Default is Home
      droneArmedStatus: "Offline",
      droneState: "",
      telemetry: null,
      interopBtnText: "Connect to Interop",
      interopBtnEnabled: true,
      moving_obstacles: [],
      stationary_obstacles: [],
      missions: []
    };
  }

  // Render based on button clicks in Navbar
  renderSelection() {
    switch(this.state.optionSelected) {
      case "Analytics":
        return (
          <Analytics appState={this.state}/>
        );
      case "Images":
        return (
          <Images appState={this.state}
                  socketEmit={this.socketEmit}/>
        );
      case "Home":
      default:
        return (
          <Home appState={this.state}
                socketEmit={this.socketEmit}/>
        );
    }
  }

  setAppState = (newState) => {
    this.setState(newState);
  }

  render() {
    return (
      <div className="App">
        <Navbar appState={this.state} setAppState={this.setAppState}
                socketEmit={this.socketEmit}/>
        {this.renderSelection()}
      </div>
    );
  }

  componentDidMount() {
    const SOCKET_DOMAIN = document.domain; // Gets domain from browser
    const SOCKET_PORT = 8084;

    const SOCKET_ADDRESS = "http://" + SOCKET_DOMAIN + ":" + SOCKET_PORT;

    this.socket = io.connect(SOCKET_ADDRESS);

    this.socket.on('connect', () => {
      console.log("Connected to ground interface feeder!");
      this.setState({
        droneArmedStatus: "Online",
        droneState: ""
      });
    });

    this.socket.on('disconnect', () => {
      console.log("Disconnected from ground interface feeder!");
      this.setState({
        droneArmedStatus: "Offline",
        droneState: ""
      });
    });

    this.socket.on('drone_connected', () => {
      this.setState({
        droneArmedStatus: "Starting Up Drone...",
        droneState: ""
      });
    });

    this.socket.on('drone_disconnected', () => {
      this.setState({
        droneArmedStatus: "Drone Disconnected!",
        droneState: ""
      });
    });

    this.socket.on('telemetry', (telemetry) => {
      // console.log(telemetry);
      this.setState({
        telemetry: telemetry,
        droneArmedStatus: telemetry["armed"] ? "Armed" : "Disarmed",
        droneState: this.convert_to_title_text(telemetry["state"])
      });
    });


    this.socket.on('image', (data) => {
      this.setState({
        testImage: data
      });
    });

    this.socket.on('initial_data', (data) => {
      this.received_interop_status(data.interop_connected);
      if (data.interop_connected) {
        this.setState({
          stationary_obstacles: data.stationary_obstacles,
          moving_obstacles: data.moving_obstacles,
          missions: data.missions
        });

        console.log("MISSION DATA!!!")
        console.log(data.missions[0]["fly_zones"][0]);
      }
    });

    this.socket.on('moving_obstacles', (moving_obstacles) => {
      this.setState({moving_obstacles: moving_obstacles});
    });

    this.socket.on('interop_connected', (is_interop_connected) =>
      this.received_interop_status(is_interop_connected)
    );
  }

  socketEmit = (message, data) => {
    if (data === undefined) {
      this.socket.emit(message);
    } else {
      this.socket.emit(message, data);
    }
  }

  received_interop_status(is_interop_connected) {
    if (is_interop_connected) {
      this.setState({
        interopBtnText: "Connected to Interop",
        interopBtnEnabled: false
      });
    } else {
      this.setState({
        interopBtnText: "Cannot Connect to Interop Server!",
        interopBtnEnabled: true
      });
      setTimeout(() => this.setState({interopBtnText: "Connect to Interop"}), 1000);
    }
  }

  convert_to_title_text(str) {
    return str.replace(/\w\S*/g, function(txt){
      return txt.charAt(0).toUpperCase() + txt.substr(1).toLowerCase();
    });
  }
}

export default App;
