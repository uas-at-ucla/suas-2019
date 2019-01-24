// use require() to load libraries from node_modules
const socketIOServer = require('socket.io');
const loadProtobufUtils = require('./protobuf_utils/protobuf_utils');
const loadInteropClient = require('./interop_client/interop_client');

const port = 8081;
const USE_FAKE_DRONE = true;
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
// loadInteropClient("192.168.2.30", 80, "testuser", "testpass")
loadInteropClient("localhost", 8000, "testuser", "testpass")
  .then(theInteropClient => {
    interopClient = theInteropClient;
  }).catch(error => {
    console.log(error);
  });

drone_io.on('connect', (socket) => {
  drone_connected = true;
  console.log("drone connected!");

  socket.on('telemetry', (data) => {
    if (protobufUtils) {
      if (data.telemetry.sensors) {
        data.telemetry.sensors = protobufUtils.decodeSensors(data.telemetry.sensors);
        if (interopClient) {
          interopClient.postTelemetry({
            latitude: data.telemetry.sensors.latitude,
            longitude: data.telemetry.sensors.longitude,
            altitude_msl: data.telemetry.sensors.altitude,
            uas_heading: data.telemetry.sensors.heading
          }).then(msg => console.log(msg));
        }
      }
      if (data.telemetry.goal) {
        data.telemetry.goal = protobufUtils.decodeGoal(data.telemetry.goal);
      }
      if (data.telemetry.output) {
        data.telemetry.output = protobufUtils.decodeOutput(data.telemetry.output);
      }
    }
    console.log(JSON.stringify(data, null, 2) + ",");
    
    // When telemetry is received from the drone, send it to clients on the UI namespace
    ui_io.emit('telemetry', data);
  });
});

fake_drone_io.on('connect', (socket) => {
  socket.on('telemetry', (data) => {
    if (!drone_connected) {
      if (data.telemetry.sensors && interopClient) {
        interopClient.postTelemetry({
          latitude: data.telemetry.sensors.latitude,
          longitude: data.telemetry.sensors.longitude,
          altitude_msl: data.telemetry.sensors.altitude,
          uas_heading: data.telemetry.sensors.heading
        }).then(msg => console.log(msg));
      }
      console.log(data);
      ui_io.emit('telemetry', data);
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