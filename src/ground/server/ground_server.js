// use require() to load libraries from node_modules
const fs = require('fs');
const socketIOServer = require('socket.io');
var ping;
try {
  ping = require("net-ping"); //might fail since it needs to be compiled specifically on each platform
} catch(e) {
  console.log(e);
  console.log("Can't load net-ping. Uninstall and reinstall for your platform with the following:");
  console.log("    cd src/ground/server");
  console.log("    npm uninstall net-ping");
  console.log("    npm install net-ping\n");
}

const loadProtobufUtils = require('./src/protobuf_utils');
const loadInteropClient = require('./src/interop_client');
const config = require('./src/config');

const port = 8081;
var droneIP = "127.0.0.1";
const USE_FAKE_DRONE = true;
const uiSendFrequency = 5; //Hz
const pingInterval = 1000 //ms
var drone_connected = false;

// create server
const io = socketIOServer(port);

// create namespaces
const ui_io = io.of('/ui');
const controls_io = io.of('/ground-controls');
const fake_drone_io = io.of('/fake-drone');

// For decoding and encoding drone messages
var protobufUtils = null;
loadProtobufUtils((theProtobufUtils) => {
  protobufUtils = theProtobufUtils;
});

var interopClient = null;
var interopData = null;
connectToInterop("134.209.2.203:8000", "testuser", "testpass", // try our test server
  (err) => {
    if (err) {
      if (fs.existsSync("/.dockerenv")) { // If inside Docker container
        connectToInterop("192.168.2.30:80", "testuser", "testpass");
      } else {
        connectToInterop("localhost:8000", "testuser", "testpass");
      }
    }
  });

function connectToInterop(ip, username, password, callback) {
  interopClient = null;
  loadInteropClient(ip, username, password)
    .then(theInteropClient => {
      interopClient = theInteropClient;
      interopClient.getMissions().then(missions =>
        interopClient.getObstacles().then(obstacles => {
          interopData = {
            ip: ip,
            mission: missions[0],
            obstacles: obstacles
          }
          ui_io.emit('INTEROP_DATA', interopData);
        })
      );
      if (callback) callback();
    }).catch(error => {
      interopData = null;
      ui_io.emit('INTEROP_DATA', interopData);
      console.log(error);
      if (callback) callback(error);
    });
}

controls_io.on('connect', (socket) => {
  drone_connected = true;
  console.log("drone connected!");
  let telemetryCount = 0;
  function onSensors(sensors, frequency) {
    let uiSendInterval = Math.floor(frequency / uiSendFrequency);
    if (protobufUtils) {
      //TODO receive and cache other data to send along with sensors
      data = {}
      data.telemetry = {}
      data.telemetry.sensors = protobufUtils.decodeSensors(sensors);
      if (interopClient) {
        interopClient.newTelemetry(data.telemetry, frequency);
      }
      // When telemetry is received from the drone, send it to clients on the UI namespace
      if (telemetryCount >= uiSendInterval) {
        if (config.verbose) console.log(JSON.stringify(data, null, 2));
        ui_io.emit('TELEMETRY', data);
        telemetryCount = 0;
      }
    }
    telemetryCount++;
  }

  socket.on('SENSORS', (sensors) => {
    onSensors(sensors, config.droneSensorsFrequency);
  });

  socket.on('SENSORS_RFD900', (sensors) => {
    onSensors(sensors, config.droneSensorsFreqRFD900);
  });
});

// FAKE DRONE
let fakeTelemetryCount = 0;
fake_drone_io.on('connect', (socket) => {
  console.log("fake drone connected!");
  let uiSendInterval = Math.floor(config.droneSensorsFrequency / uiSendFrequency);
  socket.on('TELEMETRY', (data) => {
    if (!drone_connected) { // only do stuff if the real drone is not connected
      if (interopClient) {
        interopClient.newTelemetry(data.telemetry, config.droneSensorsFrequency);
      }
      // When telemetry is received from the drone, send it to clients on the UI namespace
      if (fakeTelemetryCount >= uiSendInterval) {
        if (config.verbose) console.log(JSON.stringify(data, null, 2));
        ui_io.emit('TELEMETRY', data);
        fakeTelemetryCount = 0;
      }
      fakeTelemetryCount++;
    }
  });
});

if (ping) {
  //ping options
  var pingOptions = {
    networkProtocol: ping.NetworkProtocol.IPv4,
    packetSize: 16,
    retries: 1,
    sessionId: (process.pid % 65535),
    timeout: 3000,
    ttl: 128
  };

  try { // might fail without sudo (TODO: fix in Docker)
    var pingSession = ping.createSession(pingOptions);

    setInterval(() => pingSession.pingHost(droneIP, function (error, droneIP, sent, rcvd) {
      var ms = rcvd - sent;
      if (error)
        if (error instanceof ping.RequestTimedOutError)
          console.log(droneIP + ": Not alive");
        else
          console.log(droneIP + ": " + error.toString());
      else {
        console.log(droneIP + ": Alive (ms=" + ms + ")");
        ui_io.emit('PING', ms);
        return {
          type: 'PING',
          payload: {
            droneIP: droneIP,
            lagTime: ms
          }
        };
      }
    }), pingInterval);

    pingSession.on("close", function () {
      console.log("PING SOCKET CLOSED");
    });

    pingSession.on("error", function (error) {
      console.log(error.toString());
    });
  } catch(e) {
    console.log(e);
    console.log("Can't start ping session. You may need to run with sudo.");
  } 
}

ui_io.on('connect', (socket) => {
  console.log("ui connected!");
  if (interopData) {
    socket.emit('INTEROP_DATA', interopData);
  }

  socket.on('TEST', (data) => {
    console.log("TEST " + data);
  });

  socket.on('CHANGE_DRONE_STATE', (data) => {
    controls_io.emit('CHANGE_DRONE_STATE', data);
    console.log("THE DRONE is asked to " + data + ". Hey DRONE, are you listening?");
  });

  socket.on('RUN_MISSION', (commands) => {
    console.log("received mission from UI");
    if (protobufUtils) {
      let groundProgram = protobufUtils.makeGroundProgram(commands, interopData);
      console.log(JSON.stringify(groundProgram, null, 2));
      let encodedGroundProgram = protobufUtils.encodeGroundProgram(groundProgram);
      controls_io.emit('RUN_MISSION', encodedGroundProgram);
    }
  });

  socket.on('CONNECT_TO_INTEROP', (cred) => {
    console.log('CONNECT TO INTEROP');
    connectToInterop(cred.ip, cred.username, cred.password);
  })
});


if (USE_FAKE_DRONE) {
  require('./src/fake_drone/fake_drone');
}