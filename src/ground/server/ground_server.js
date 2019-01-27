// use require() to load libraries from node_modules
const fs = require('fs');
const socketIOServer = require('socket.io');
const loadProtobufUtils = require('./protobuf_utils/protobuf_utils');
const loadInteropClient = require('./interop_client/interop_client');
const constants = require('./utils/constants');

const port = 8081;
const USE_FAKE_DRONE = true;
const uiSendFrequency = 5; //Hz
const uiSendInterval = Math.floor(constants.droneTelemetryFrequency / uiSendFrequency);
var drone_connected = false;

// create server
const io = socketIOServer(port);

// create namespaces
const ui_io = io.of('/ui');
const drone_io = io.of('/drone');
const fake_drone_io = io.of('/fake-drone');

// For decoding and encoding drone messages
var protobufUtils = null;
loadProtobufUtils((theProtobufUtils) => {
  protobufUtils = theProtobufUtils;
});

var interopClient = null;
(fs.existsSync("/.dockerenv") // If inside Docker container
  ? loadInteropClient("192.168.2.30", 80, "testuser", "testpass")
  : loadInteropClient("localhost", 8000, "testuser", "testpass")
).then(theInteropClient => {
  interopClient = theInteropClient;
}).catch(error => {
  console.log(error);
});

drone_io.on('connect', (socket) => {
  drone_connected = true;
  console.log("drone connected!");

  telemetryCount = 0;
  socket.on('telemetry', (data) => {
    if (protobufUtils) {
      data.telemetry = protobufUtils.decodeTelemetry(data.telemetry);
      if (interopClient) {
        interopClient.newTelemetry(data.telemetry);
      }
      // When telemetry is received from the drone, send it to clients on the UI namespace
      if (telemetryCount >= uiSendInterval) {
        console.log(JSON.stringify(data, null, 2) + ",");
        ui_io.emit('telemetry', data);
        telemetryCount = 0;
      }
    }
    telemetryCount++;
  });
});

// FAKE DRONE
fakeTelemetryCount = 0;
fake_drone_io.on('connect', (socket) => {
  console.log("fake drone connected!");
  socket.on('telemetry', (data) => {
    if (!drone_connected) { // only do stuff if the real drone is not connected
      if (interopClient) {
        interopClient.newTelemetry(data.telemetry);
      }
      // When telemetry is received from the drone, send it to clients on the UI namespace
      if (fakeTelemetryCount >= uiSendInterval) {
        console.log(JSON.stringify(data, null, 2) + ",");
        ui_io.emit('telemetry', data);
        fakeTelemetryCount = 0;
      }
      fakeTelemetryCount++;
    }
  });
});

ui_io.on('connect', (socket) => {
  console.log("ui connected!");

  socket.on("TEST", (data) => {
    console.log("TEST " + data);
  });
  socket.on("CHANGE_DRONE_STATE", (data) => {
    drone_io.emit("CHANGE_DRONE_STATE", data);
    console.log("THE DRONE is asked to " + data + ". Hey DRONE, are you listening?");
  });
});


if (USE_FAKE_DRONE) {
  require('./fake_drone/fake_drone');
}