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

// Temporarily commented
// // For decoding and encoding drone messages
// var protobufUtils = null;
// loadProtobufUtils((theProtobufUtils) => {
//   protobufUtils = theProtobufUtils;
// });

var interopClient = null;
var missionAndObstacles = null;
connectToInterop("134.209.2.203", 8000, "testuser", "testpass", // try our test server
/*onError*/() => {
  if (fs.existsSync("/.dockerenv")) { // If inside Docker container
    connectToInterop("192.168.2.30", 80, "testuser", "testpass");
  } else {
    connectToInterop("localhost", 8000, "testuser", "testpass");
  }
});

function connectToInterop(ip, port, username, password, onError) {
  interopClient = null;
  loadInteropClient(ip, port, username, password)
    .then(theInteropClient => {
      interopClient = theInteropClient;
      interopClient.getMissions().then(missions =>
        interopClient.getObstacles().then(obstacles => {
          missionAndObstacles = {
            mission: missions[0],
            obstacles: obstacles
          }
          ui_io.emit('INTEROP_DATA', missionAndObstacles);
        })
      );
    }).catch(error => {
      console.log(error);
      onError();
    });
}

drone_io.on('connect', (socket) => {
  drone_connected = true;
  console.log("drone connected!");

  telemetryCount = 0;
  socket.on('TELEMETRY', (data) => {
    if (protobufUtils) {
      data.telemetry = protobufUtils.decodeTelemetry(data.telemetry);
      if (interopClient) {
        interopClient.newTelemetry(data.telemetry);
      }
      // When telemetry is received from the drone, send it to clients on the UI namespace
      if (telemetryCount >= uiSendInterval) {
        if (constants.verbose) console.log(JSON.stringify(data, null, 2));
        ui_io.emit('TELEMETRY', data);
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
  socket.on('TELEMETRY', (data) => {
    if (!drone_connected) { // only do stuff if the real drone is not connected
      if (interopClient) {
        interopClient.newTelemetry(data.telemetry);
      }
      // When telemetry is received from the drone, send it to clients on the UI namespace
      if (fakeTelemetryCount >= uiSendInterval) {
        if (constants.verbose) console.log(JSON.stringify(data, null, 2));
        ui_io.emit('TELEMETRY', data);
        fakeTelemetryCount = 0;
      }
      fakeTelemetryCount++;
    }
  });
});

ui_io.on('connect', (socket) => {
  console.log("ui connected!");
  if (missionAndObstacles) {
    socket.emit('INTEROP_DATA', missionAndObstacles);
  }

  socket.on('TEST', (data) => {
    console.log("TEST " + data);
  });
  socket.on('CHANGE_DRONE_STATE', (data) => {
    drone_io.emit('CHANGE_DRONE_STATE', data);
    console.log("THE DRONE is asked to " + data + ". Hey DRONE, are you listening?");
  });

  socket.on('RUN_MISSION', (commands) => {
    console.log("received mission from UI");
    if (protobufUtils) {
      let groundProgram = protobufUtils.makeGroundProgram(commands, missionAndObstacles);
      console.log(JSON.stringify(groundProgram, null, 2));
      let encodedGroundProgram = protobufUtils.encodeGroundProgram(groundProgram);
      drone_io.emit('RUN_MISSION', encodedGroundProgram);
    }
  });
});


if (USE_FAKE_DRONE) {
  require('./fake_drone/fake_drone');
}