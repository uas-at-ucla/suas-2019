// use require() to load libraries from node_modules
const fs = require('fs');
const socketIOServer = require('socket.io');
var ping;
try {
  ping = require("net-ping"); //might fail since it needs to be compiled specifically on each platform
} catch(e) {
  console.log(e);
  console.log("Can't load net-ping. It either did not install succesfully, or it may be installed for the wrong platform. Uninstall and reinstall on the desired platform with:");
  console.log("    cd src/ground/server");
  console.log("    npm uninstall net-ping --no-save");
  console.log("    npm install net-ping\n");
}

const loadProtobufUtils = require('./src/protobuf_utils');
const loadInteropClient = require('./src/interop_client');
const config = require('./config');

const server_port = 8081;
var droneIP = config.testing ? "192.168.3.20" : "192.168.1.20";
const pingInterval = 1000 //ms
const uiSendFrequency = 5; //Hz
const trackySendFrequency = 5; //Hz
var telemetry = {};

var drone_connected = false;

// create server
const io = socketIOServer(server_port);

// create namespaces
const ui_io = io.of('/ui');
const controls_io = io.of('/ground-controls');
const ugv_io = io.of('/ugv');
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
var interopClient = null;
var interopData = null;
if (config.testing) {
  // try our test server
  connectToInterop("134.209.2.203:8000", "testuser", "testpass", 2)
    .catch(error => {
      if (fs.existsSync("/.dockerenv")) { // If inside Docker container
        connectToInterop("192.168.3.30:80", "testuser", "testpass", 2);
      } else {
        connectToInterop("localhost:8000", "testuser", "testpass", 2);
      }
    });
}

function connectToInterop(ip, username, password, missionId) {
  interopClient = null;
  return loadInteropClient(ip, username, password, ui_io)
    .then(theInteropClient => {
      interopClient = theInteropClient;
      interopClient.getMission(missionId).then(mission => {
        interopData = {
          ip: ip,
          mission: mission
        }
        console.log("Interop data retrieved");
        ui_io.emit('INTEROP_DATA', interopData);
      }).catch(error => {
        console.log("Interop mission retrieval failed. Check the mission ID.");
        interopData = null;
        ui_io.emit('INTEROP_DATA', interopData);
        if (config.verbose) console.log(error);
      });
    }).catch(error => {
      console.log("Interop login failed");
      interopData = null;
      ui_io.emit('INTEROP_DATA', interopData);
      if (config.verbose) console.log(error);
    });
}

/**************************
 * ANTENNA TRACKER
 **************************/
const trackySendInterval = 1000 / trackySendFrequency;
setInterval(() => {
  if (telemetry.sensors) {
    tracky_io.emit('DRONE_POS', telemetry.sensors);
  }
}, trackySendInterval);
tracky_io.on('connect', (socket) => {
  console.log("Antenna tracker connected!");
});


/**************************
 * GROUND_CONTROLS SOCKET
 **************************/
controls_io.on('connect', (socket) => {
  drone_connected = true;
  // droneIP = socket.handshake.address.replace('::ffff:', '');
  console.log("ground_controls connected");
  function onSensors(sensors) {
    if (protobufUtils) {
      telemetry.sensors = protobufUtils.decodeSensors(sensors);
      if (interopClient) {
        interopClient.newTelemetry(telemetry);
      }
    }
  }

  socket.on('SENSORS', (sensors) => {
    onSensors(sensors);
  });

  socket.on('SENSORS_RFD900', (sensors) => {
    onSensors(sensors);
  });

  socket.on('COMPILED_DRONE_PROGRAM', (droneProgram) => {
    if (protobufUtils) {
      ui_io.emit('COMPILED_DRONE_PROGRAM', protobufUtils.decodeDroneProgam(droneProgram));
    }
  });

  socket.on('UPLOADED_DRONE_PROGRAM', (droneProgram) => {
    console.log("Got acknowledgment that the Drone Program was uploaded")
    if (protobufUtils) {
      ui_io.emit('UPLOADED_DRONE_PROGRAM', protobufUtils.decodeDroneProgam(droneProgram));
    }  
  });

  let msgs_to_ui = [
    'MISSION_COMPILE_ERROR', 'MISSION_STATUS',
    'GIMBAL_SETPOINT',
    'DEPLOYMENT_MOTOR_SETPOINT',
    'LATCH_SETPOINT',
    'HOTWIRE_SETPOINT'
  ];
  for (let ui_msg of msgs_to_ui) {
    let local_ui_msg = ui_msg;
    socket.on(local_ui_msg, (data) => {
      console.log("received: " + local_ui_msg + ": " + data);
      ui_io.emit(local_ui_msg, data);
    });
  }
});


/**************************
 * UGV SOCKET
 **************************/
ugv_io.on('connect', (socket) => {
  console.log("UGV controls connected!");
  socket.emit('SET_TARGET', {lat: 38.14617, lng: -76.42642}); // Official competition destination

  socket.on('UGV_MESSAGE', (msg) => {
    if (protobufUtils) {
      msg = protobufUtils.decodeUGV_Message(msg);
      console.log(msg);
      ui_io.emit('UGV_MESSAGE', msg);
    }
  });
});


/**************************
 * UI SOCKET
 **************************/
const uiSendInterval = 1000 / uiSendFrequency;
setInterval(() => { // periodically send telemetry to UI
  if (telemetry.sensors) {
    if (config.verbose) console.log(JSON.stringify(telemetry, null, 2));
    ui_io.emit('TELEMETRY', telemetry);
  }
}, uiSendInterval);
ui_io.on('connect', (socket) => {
  console.log("ui connected!");
  if (interopData) {
    socket.emit('INTEROP_DATA', interopData);
  }

  socket.on('TEST', (data) => {
    console.log("TEST " + data);
  });

  socket.on('COMPILE_GROUND_PROGRAM', (commands) => {
    console.log("received ground program from UI");
    if (protobufUtils) {
      let groundProgram = protobufUtils.makeGroundProgram(commands, interopData);
      console.log(JSON.stringify(groundProgram, null, 2));
      let encodedGroundProgram = protobufUtils.encodeGroundProgram(groundProgram);
      console.log("Sending ground program to the drone");
      controls_io.emit('COMPILE_GROUND_PROGRAM', encodedGroundProgram);
    }
  });

  let msgs_to_drone = [
    'UPLOAD_MISSION', 'RUN_MISSION', 'PAUSE_MISSION', 'END_MISSION',
    'CHANGE_DRONE_STATE', 
    'GIMBAL_SETPOINT',
    'DEPLOYMENT_MOTOR_SETPOINT',
    'LATCH_SETPOINT',
    'HOTWIRE_SETPOINT'
  ];
  for (let controls_msg of msgs_to_drone) {
    let local_controls_msg = controls_msg;
    socket.on(local_controls_msg, (data) => {
      console.log("sending: " + local_controls_msg + ": " + data);
      controls_io.emit(local_controls_msg, data);
    });
  }

  socket.on('CONNECT_TO_INTEROP', (cred) => {
    console.log('CONNECT TO INTEROP');
    connectToInterop(cred.ip, cred.username, cred.password, cred.missionId);
  });

  socket.on('CONFIGURE_TRACKY_POS', (pos) => {
    console.log("Sending Tracky its estimated position");
    tracky_io.emit('CONFIGURE_POS', pos);
  });

  socket.on('SET_UGV_TARGET', (pos) => {
    console.log("Sending the UGV its target position");
    ugv_io.emit('SET_TARGET', pos);
  });

  socket.on('DRIVE_UGV', () => {
    console.log("Driving the UGV!");
    ugv_io.emit('DRIVE_TO_TARGET'); // TODO automatically send to UGV when it hits the ground
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
fake_drone_io.on('connect', (socket) => {
  console.log("fake drone connected!");
  socket.on('TELEMETRY', (droneTelemetry) => {
    if (!drone_connected) { // only do stuff if the real drone is not connected
      telemetry = droneTelemetry;
      if (interopClient) {
        interopClient.newTelemetry(telemetry);
      }
    }
  });
});


if (config.testing && config.useFakeDrone) {
  require('./src/fake_drone/fake_drone');
}
