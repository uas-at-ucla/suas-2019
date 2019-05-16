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
const config = require('./config');

const USE_FAKE_DRONE = true;
const server_port = 8081;
var droneIP = "192.168.1.20";
const pingInterval = 1000 //ms
const uiSendFrequency = 5; //Hz

var drone_connected = false;

// create server
const io = socketIOServer(server_port);

// create namespaces
const ui_io = io.of('/ui');
const controls_io = io.of('/ground-controls');
const tracky_io = io.of('/tracky');
const fake_drone_io = io.of('/fake-drone');

// For decoding and encoding drone messages
var protobufUtils = null;
loadProtobufUtils((theProtobufUtils) => {
  protobufUtils = theProtobufUtils;
});


/**************************
 * INTEROP CONNECTION
 **************************/
var interopCanUpload = true;
var interopClient = null;
var interopData = null;
if (config.testing) {
  // try our test server
  connectToInterop("134.209.2.203:8000", "testuser", "testpass")
    .catch(error => {
      if (fs.existsSync("/.dockerenv")) { // If inside Docker container
        connectToInterop("192.168.2.30:80", "testuser", "testpass");
      } else {
        connectToInterop("localhost:8000", "testuser", "testpass");
      }
    });
}

function connectToInterop(ip, username, password) {
  interopClient = null;
  return loadInteropClient(ip, username, password)
    .then(theInteropClient => {
      interopClient = theInteropClient;
      interopClient.getMissions().then(missions =>
        interopClient.getObstacles().then(obstacles => {
          interopData = {
            ip: ip,
            mission: missions[0],
            obstacles: obstacles
          }
          console.log("Interop data retrieved");
          ui_io.emit('INTEROP_DATA', interopData);
        })
      );
    }).catch(error => {
      interopData = null;
      ui_io.emit('INTEROP_DATA', interopData);
      throw error;
    });
}


/**************************
 * GROUND_CONTROLS SOCKET
 **************************/
const uiSendInterval = {
  [config.droneSensorsFrequency]: Math.floor(config.droneSensorsFrequency / uiSendFrequency),
  [config.droneSensorsFreqRFD900]: Math.floor(config.droneSensorsFreqRFD900 / uiSendFrequency)
}
controls_io.on('connect', (socket) => {
  drone_connected = true;
  // droneIP = socket.handshake.address.replace('::ffff:', '');
  console.log("ground_controls connected");
  let telemetryCount = 0;
  function onSensors(sensors, frequency) {
    if (protobufUtils) {
      //TODO receive and cache other data to send along with sensors
      telemetry = {}
      telemetry.sensors = protobufUtils.decodeSensors(sensors);
      if (interopClient) {
        interopClient.newTelemetry(telemetry, frequency).then(() => {
          if (!interopCanUpload) {
            interopCanUpload = true;
            ui_io.emit('INTEROP_UPLOAD_SUCCESS');
          }
        }).catch(() => {
          if (interopCanUpload) {
            interopCanUpload = false;
            ui_io.emit('INTEROP_UPLOAD_FAIL');
          }
        });
      }
      // When telemetry is received from the drone, send it to clients on the UI namespace
      if (telemetryCount >= uiSendInterval[frequency]) {
        if (config.verbose) console.log(JSON.stringify(telemetry, null, 2));
        ui_io.emit('TELEMETRY', telemetry);
        tracky_io.emit('DRONE_POS', telemetry.sensors);
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


/**************************
 * UI SOCKET
 **************************/
ui_io.on('connect', (socket) => {
  console.log("ui connected!");
  if (interopData) {
    socket.emit('INTEROP_DATA', interopData);
  }

  socket.on('TEST', (data) => {
    console.log("TEST " + data);
  });

  socket.on('CHANGE_DRONE_STATE', (state) => {
    controls_io.emit('CHANGE_DRONE_STATE', state);
    console.log("THE DRONE is asked to " + state + ". Hey DRONE, are you listening?");
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
  });

  socket.on('CONFIGURE_TRACKY_POS', (pos) => {
    console.log("Sending Tracky its estimated position");
    tracky_io.emit('CONFIGURE_POS', pos);
  });
});


/**************************
 * PING THE DRONE
 **************************/
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

  try { // might fail without sudo
    var pingSession = ping.createSession(pingOptions);

    setInterval(() => droneIP && pingSession.pingHost(droneIP, function (error, droneIP, sent, rcvd) {
      var ms = rcvd - sent;
      if (error) {
        ui_io.emit('PING', null);
        if (error instanceof ping.RequestTimedOutError)
          console.log(droneIP + ": Not alive");
        else
          console.log(droneIP + ": " + error.toString());
      } else {
        if (config.verbose) console.log(droneIP + ": Alive (ms=" + ms + ")");
        ui_io.emit('PING', ms);
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
    console.log("Can't start ping session. You may need to run with sudo\n.");
  }
}

/**************************
 * FAKE_DRONE SOCKET
 **************************/
let fakeTelemetryCount = 0;
fake_drone_io.on('connect', (socket) => {
  console.log("fake drone connected!");
  socket.on('TELEMETRY', (telemetry) => {
    if (!drone_connected) { // only do stuff if the real drone is not connected
      if (interopClient) {
        interopClient.newTelemetry(telemetry, config.droneSensorsFrequency).then(() => {
          if (!interopCanUpload) {
            interopCanUpload = true;
            ui_io.emit('INTEROP_UPLOAD_SUCCESS');
          }
        }).catch(() => {
          if (interopCanUpload) {
            interopCanUpload = false;
            ui_io.emit('INTEROP_UPLOAD_FAIL');
          }
        });
      }
      // When telemetry is received from the drone, send it to clients on the UI namespace
      if (fakeTelemetryCount >= uiSendInterval[config.droneSensorsFrequency]) {
        if (config.verbose) console.log(JSON.stringify(telemetry, null, 2));
        ui_io.emit('TELEMETRY', telemetry);
        tracky_io.emit('DRONE_POS', telemetry.sensors);
        fakeTelemetryCount = 0;
      }
      fakeTelemetryCount++;
    }
  });
});


if (USE_FAKE_DRONE) {
  require('./src/fake_drone/fake_drone');
}
