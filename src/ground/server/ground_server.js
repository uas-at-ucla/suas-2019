// use require() to load libraries from node_modules
const socketIOServer = require('socket.io');
const loadProtobufUtils = require('./protobuf_utils/protobuf_utils');
const loadInteropClient = require('./interop_client/interop_client');

const port = 8081;

// create server
const io = socketIOServer(port);

// create two namespaces
const drone_io = io.of('/drone');
const ui_io = io.of('/ui');

// For decoding and encoding drone messages
var protobufUtils = null;
loadProtobufUtils((theProtobufUtils) => {
  protobufUtils = theProtobufUtils;
});

var interopClient = null;
loadInteropClient("localhost", 8000, "testadmin", "testpass")
  .then(theInteropClient => {
    interopClient = theInteropClient;
  }).catch(error => {
    console.log(error);
  });

drone_io.on('connect', (socket) => {
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
          });
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


// // Fake Drone
// const socketIOClient = require('socket.io-client');
// const socket = socketIOClient('http://localhost:'+port+'/drone', { transports: ['websocket'] });

// let telemetryNumber = 0;
// setInterval(() => {
//   socket.emit('telemetry', "Fake Telemetry " + telemetryNumber);
//   telemetryNumber++;
// }, 1000);
