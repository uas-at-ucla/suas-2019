// use require() to load libraries from node_modules
const socketIOServer = require('socket.io');

const port = 8081;

// create server
const io = socketIOServer(port);

// create two namespaces
const drone_io = io.of('/drone');
const ui_io = io.of('/ui');

drone_io.on('connect', (socket) => {
  console.log("drone connected!");

  socket.on('telemetry', (data) => {
    // console.log("Received Telemetry: " + data);
    // When telemetry is received from the drone, send it to clients on the UI namespace
    ui_io.emit('telemetry', data);
  });
});

ui_io.on('connect', (socket) => {
  console.log("ui connected!");

  socket.on("TEST", (data) =>{
    console.log("TEST " + data);
  })
  socket.on("CHANGE_DRONE_STATE", (data) =>{
    console.log("THE DRONE is asked to " + data + ". THE DRONE says no.")
  })
});


// Fake Drone
const socketIOClient = require('socket.io-client');
const socket = socketIOClient('http://localhost:'+port+'/drone', { transports: ['websocket'] });

let telemetryNumber = 0;
setInterval(() => {
  socket.emit('telemetry', "Fake Telemetry " + telemetryNumber);
  telemetryNumber++;
}, 1000);
