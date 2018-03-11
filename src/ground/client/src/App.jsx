import React, { Component } from "react";
import Navbar from "./Navbar/Navbar";
import "./App.css";
import io from "socket.io-client/dist/socket.io.js";
import logo from "./graphics/vector_logo.svg";

import Home from "./Home/Home";
import Analytics from "./Analytics/Analytics";
import Images from "./Images/Images";
import Settings from "./Settings/Settings";
import Training from "./Training/Training";

class App extends Component {
  constructor(props) {
    super(props);

    this.state = {
      optionSelected: "Control", // Default is Control
      droneState: "Offline",
      telemetry: null,
      interopBtnText: "Connect to Interop",
      interopBtnEnabled: true,
      moving_obstacles: [],
      stationary_obstacles: [],
      missions: [],
      followDrone: true
    };
  }

  // Render based on button clicks in Navbar
  renderSelection() {
    switch (this.state.optionSelected) {
      case "Analytics":
        return <Analytics appState={this.state} />;
      case "Images":
        return <Images appState={this.state} socketEmit={this.socketEmit} />;
      case "Training":
        return <Training appState={this.state} socketEmit={this.socketEmit} />;
      case "Settings":
        return <Settings appState={this.state} socketEmit={this.socketEmit}
                         setAppState={this.setAppState}/>;
      case "Control":
      default:
        return <Home appState={this.state} socketEmit={this.socketEmit}
                     setAppState={this.setAppState}/>;
    }
  }

  setAppState = newState => {
    this.setState(newState);
  };

  render() {
    return (
      <div className="App">
        <img
          id="logo"
          src={logo}
          width="380px"
          onClick={this.followDrone}
          alt="UAS"
        />
        <Navbar
          appState={this.state}
          setAppState={this.setAppState}
          socketEmit={this.socketEmit}
        />
        {this.renderSelection()}
      </div>
    );
  }

  followDrone = () => {
    if (this.state.optionSelected === "Control") {
      this.setState({ followDrone: true });
    }
  }

  componentDidMount() {
    const SOCKET_DOMAIN = document.domain; // Gets domain from browser
    const SOCKET_PORT = 8084;

    const SOCKET_ADDRESS = "http://" + SOCKET_DOMAIN + ":" + SOCKET_PORT;

    this.socket = io.connect(SOCKET_ADDRESS);

    this.socket.on("connect", () => {
      console.log("Connected to ground interface feeder!");
      this.setState({
        droneState: "Ground Online",
      });
    });

    this.socket.on("disconnect", () => {
      console.log("Disconnected from ground interface feeder!");
      this.setState({
        droneState: "Ground Offline",
      });
    });

    this.socket.on("drone_connected", () => {
      this.setState({
        droneState: "Starting Up Drone...",
      });
    });

    this.socket.on("drone_disconnected", () => {
      this.setState({
        droneState: "Drone Disconnected!",
      });
    });

    this.socket.on("telemetry", telemetry => {
      // console.log(telemetry);
      let newState = {
        telemetry: telemetry
      }
      if (telemetry.status && telemetry.status.state) {
        newState.droneState = this.convertToTitleText(telemetry.status.state)
      }
      this.setState(newState);
    });

    this.socket.on("image", data => {
      this.setState({
        testImage: data
      });
    });

    this.socket.on("initial_data", data => {
      if (data.interop_disconnected) {
        this.receivedInteropStatus(false);
      } else {
        this.setState({
          stationary_obstacles: data.stationary_obstacles,
          moving_obstacles: data.moving_obstacles,
          missions: data.missions
        });
        this.receivedInteropStatus(true);

        console.log("MISSION DATA!!!");
        console.log(data.missions[0]);
      }
    });

    this.socket.on("interop_data", data => {
      this.setState(data);
      this.receivedInteropStatus(true);
    });

    this.socket.on("interop_disconnected", () => {
      this.receivedInteropStatus(false);
    });
  }

  socketEmit = (message, data) => {
    if (data === undefined) {
      this.socket.emit(message);
    } else {
      this.socket.emit(message, data);
    }
  };

  receivedInteropStatus(is_interop_connected) {
    if (is_interop_connected) {
      let new_state = {
        interopBtnText: "Connected to Interop",
        interopBtnEnabled: false
      }
      for (let key in new_state) {
        if (new_state[key] !== this.state[key]) {
          this.setState(new_state);
          break;
        }
      }
    } else {
      this.setState({
        interopBtnText: "Cannot Connect to Interop Server!",
        interopBtnEnabled: true
      });
      setTimeout(
        () =>
          this.setState({
            interopBtnText: "Connect to Interop"
          }),
        1000
      );
    }
  }

  convertToTitleText(str) {
    return str.replace(/\w\S*/g, function(txt) {
      return txt.charAt(0).toUpperCase() + txt.substr(1).toLowerCase();
    });
  }
}

export default App;
