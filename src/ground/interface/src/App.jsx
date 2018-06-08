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

const photoFolder = 'testPhotos'

class App extends Component {
  constructor(props) {
    super(props);

    this.state = {
      optionSelected: "Control", // Default is Control
      droneState: "Offline",
      telemetry: null,
      drone_ping_ms: null,
      interopBtnText: "Connect to Interop",
      interopBtnEnabled: true,
      moving_obstacles: [],
      stationary_obstacles: [],
      missions: [],
      followDrone: true,
      metric: true,
      photoFolder: photoFolder,
      rawImages: [],
      segmentedImages: [],
      autoClassifiedImages: [],
      croppedImages: {},
      manualCroppedImageParents: [],
      manualClassifiedImages: []
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
                     setAppState={this.setAppState} socketOn={this.socketOn} />;
    }
  }

  setAppState = newState => {
    this.setState(newState);
  };

  render() {
    return (
      <div className={`App ${this.state.metric ? "metric_container" : "imperial_container"}`}>
        {this.state.optionSelected !== "Images" ?
          <img
            id="logo"
            src={logo}
            width="380px"
            onClick={this.followDrone}
            alt="UAS"
          /> : null
        }
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

  componentWillMount() {
    const SOCKET_DOMAIN = document.domain; // Gets domain from browser
    const SOCKET_PORT = 8081;

    const SOCKET_ADDRESS = "http://" + SOCKET_DOMAIN + ":" + SOCKET_PORT;

    this.socket = io.connect(SOCKET_ADDRESS);
  }

  componentDidMount() {
    this.socket.on("connect", () => {
      console.log("Connected to ground interface feeder!");
      this.socket.emit("join_room", "frontend");

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
        droneState: "Drone Connected!",
      });
    });

    this.socket.on("drone_disconnected", () => {
      this.setState({
        droneState: "Drone Disconnected!",
        telemetry: null,
        drone_mission_base64: null
      });
    });

    this.socket.on("drone_ping", (data) => {
      this.setState({drone_ping_ms: data.ms});
    });

    this.socket.on("initial_data", data => {
      let rawImages = []
      for (let id of data.all_images.raw) {
        rawImages.push({
          id: id,
          src: '/'+photoFolder+'/' + id + '.JPG'
        });
      }
      let segmentedImages = []
      for (let id of data.all_images.localized) {
        fetch('/'+photoFolder+'/' + id + '.json')
          .then(res => res.json())
          .catch(error => console.log("No JSON file exists!"))
          .then(json_data => {
            console.log(json_data)
            let croppedImages = Object.assign({}, this.state.croppedImages);
            croppedImages[id] = json_data.parent_img_id;
            this.setState({croppedImages: croppedImages});
          })
          .catch(error => console.log("Fetch request failed."));
        segmentedImages.push({
          id: id,
          src: '/'+photoFolder+'/' + id + '.JPG'
        })
      }
      let autoClassifiedImages = []
      for (let id of data.all_images.classified) {
        autoClassifiedImages.push(id);
      }
      this.setState({
        rawImages: rawImages,
        segmentedImages: segmentedImages,
        autoClassifiedImages: autoClassifiedImages,
        manualClassifiedImages: data.manual_classified_images,
        manualCroppedImageParents: data.manual_cropped_images
      });

      if (data.drone_connected) {
        this.setState({droneState: "Drone Connected!"});
      }

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
      }
    });

    this.socket.on("on_telemetry", telemetry => {
      let newState = {
        telemetry: telemetry.telemetry
      }
      if (telemetry.mission !== this.state.drone_mission_base64) {
        newState.drone_mission_base64 = telemetry.mission;
      }
      telemetry = telemetry.telemetry;
      if (telemetry.status && telemetry.status.state) {
        newState.droneState = this.convertToTitleText(telemetry.status.state);
      }

      this.setState(newState);
    });

    this.socket.on("added_images", data => {
      for (let id of data.raw) {
        this.state.rawImages.push({
          id: id,
          src: '/'+photoFolder+'/' + id + '.JPG'
        });
      }
      for (let id of data.localized) {
        fetch('/'+photoFolder+'/' + id + '.json')
          .then(res => res.json())
          .catch(error => console.log("No JSON file exists!"))
          .then(json_data => {
            let croppedImages = Object.assign({}, this.state.croppedImages);
            croppedImages[id] = json_data.parent_img_id;
            this.setState({croppedImages: croppedImages});
          })
          .catch(error => console.log("Fetch request failed."));

        this.state.segmentedImages.push({
          id: id,
          src: '/'+photoFolder+'/' + id + '.JPG'
        });
      }
      for (let id of data.classified) {
        this.state.autoClassifiedImages.push(id);
      }
      this.setState({});
    });

    this.socket.on("cropped_image", id => {
      let manualCroppedImageParents = this.state.manualCroppedImageParents.slice()
      manualCroppedImageParents.push(id);
      this.setState({manualCroppedImageParents: manualCroppedImageParents});
    });

    this.socket.on("classified_image", data => {
      let manualClassifiedImages = this.state.manualClassifiedImages.slice()
      manualClassifiedImages.push(data);
      this.setState({manualClassifiedImages: manualClassifiedImages});
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

  socketOn = (message, func) => {
    this.socket.on(message, func);
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
        interopBtnEnabled: true,
        stationary_obstacles: [],
        moving_obstacles: [],
        missions: []
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
    return str.replace('_', ' ').replace(/\w\S*/g, function(txt) {
      return txt.charAt(0).toUpperCase() + txt.substr(1).toLowerCase();
    });
  }
}

export default App;
