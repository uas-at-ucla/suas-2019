// use require() to load libraries from node_modules
const socketIOServer = require('socket.io');
const loadProtobufUtils = require('./protobuf_utils/protobuf_utils');

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

drone_io.on('connect', (socket) => {
  console.log("drone connected!");

  socket.on('telemetry', (data) => {
    if (protobufUtils) {
      if (data.telemetry.sensors) {
        data.telemetry.sensors = protobufUtils.decodeSensors(data.telemetry.sensors);
      }
      if (data.telemetry.status) {
        data.telemetry.status = protobufUtils.decodeStatus(data.telemetry.status);
      }
      if (data.telemetry.goal) {
        data.telemetry.goal = protobufUtils.decodeGoal(data.telemetry.goal);
      }
      if (data.telemetry.output) {
        data.telemetry.output = protobufUtils.decodeOutput(data.telemetry.output);
      }
    }
    console.log("Received Telemetry: " + JSON.stringify(data));
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
    console.log("THE DRONE is asked to " + data + ". THE DRONE says no.");
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
