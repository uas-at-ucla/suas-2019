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

const inDockerContainer = fs.existsSync("/.dockerenv");

const loadProtobufUtils = require('./src/protobuf_utils');
const loadInteropClient = require('./src/interop_client');
const config = require('./config');

const server_port = 8081;

var droneIP = inDockerContainer ? "192.168.3.20" : "192.168.1.20";
const pingInterval = 1000 //ms
const uiSendFrequency = 5; //Hz
const trackySendFrequency = 5; //Hz
var telemetry = { output: {} };
var flightControllerState = null;
var droneArmed = null;
var drone_connected = false;

// const ugvWaitAfterUnlatchTime = 15000; //ms
// const ugvStillTimeThreshold = 15000; //ms
const ugvWaitTimeAfterCut = 15000; // ms
var droppyReady = false;
// var dropping = false;
// var ugvUnlatchTime = 0;
// var ugvIsStill = false;
// var ugvBecameStillTime = 0;
var droveUgv = false;
var ugvDriveTimer = null;


// create server
const io = socketIOServer(server_port);

// create namespaces
const ui_io = io.of('/ui');
const controls_io = io.of('/ground-controls');
const ugv_io = io.of('/ugv');
// const tracky_io = io.of('/tracky');
const button_panel_io = io.of('/button-panel');
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
      if (inDockerContainer) {
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
// const trackySendInterval = 1000 / trackySendFrequency;
// setInterval(() => {
//   if (telemetry.sensors) {
//     tracky_io.emit('DRONE_POS', telemetry.sensors);
//   }
// }, trackySendInterval);
// tracky_io.on('connect', (socket) => {
//   console.log("Antenna tracker connected!");
// });


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
      if (telemetry.sensors.autopilot_state !== flightControllerState || telemetry.sensors.armed !== droneArmed) {
        flightControllerState = telemetry.sensors.autopilot_state;
        droneArmed = telemetry.sensors.armed;
        if (telemetry.sensors.armed) {
          button_panel_io.emit('DRONE_STATE', flightControllerState);
        } else {
          button_panel_io.emit('DRONE_STATE', "DISARMED");
        }
      }
    }
  }

  socket.on('SENSORS', (sensors) => {
    onSensors(sensors);
  });

  socket.on('SENSORS_RFD900', (sensors) => {
    onSensors(sensors);
  });

  socket.on('OUTPUT', (output) => {
    if (protobufUtils) {
      telemetry.output = protobufUtils.decodeOutput(output);
      if (telemetry.output.deploy !== droppyReady) {
        droppyReady = telemetry.output.deploy;
        button_panel_io.emit('DROPPY_READY', droppyReady);
      }
    }
  });

  socket.on('COMPILED_DRONE_PROGRAM', (droneProgram) => {
    if (protobufUtils) {
      droneProgram = protobufUtils.decodeDroneProgam(droneProgram);
      console.log(droneProgram)
      console.log("Received drone program^");
      ui_io.emit('COMPILED_DRONE_PROGRAM', droneProgram);
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
    'HOTWIRE_SETPOINT',
    'DROPPY_COMMAND_RECEIVED'
  ];
  for (let ui_msg of msgs_to_ui) {
    let local_ui_msg = ui_msg;
    socket.on(local_ui_msg, (data) => {
      console.log("received: " + local_ui_msg + ": " + data);
      ui_io.emit(local_ui_msg, data);

      if (local_ui_msg === 'DROPPY_COMMAND_RECEIVED') {
        if (data === 'RESET_LATCH') {
          droveUgv = false;
        } else if (data === 'STOP_CUT' || data === 'CANCEL_DROP') {
          if (ugvDriveTimer) {
            clearTimeout(ugvDriveTimer);
            droveUgv = false;
          }
        } if (data === 'CUT_LINE' && !droveUgv) {
          droveUgv = true;
          console.log("Going to drive UGV in "+(ugvWaitTimeAfterCut/1000)+" seconds (unless cancelled)");
          ugvDriveTimer = setTimeout(() => {
            console.log("Driving the UGV!");
            ugv_io.emit('DRIVE_TO_TARGET');
          }, ugvWaitTimeAfterCut);
        } /*else if (data === 'START_DROP') {
          dropping = true;
          ugvUnlatchTime = Date.now();
        }*/

        button_panel_io.emit('DROPPY_COMMAND_RECEIVED', data);
      }
    });
  }

  socket.on('disconnect', () => {
    console.log("ground_controls disconnected");
  });
});


/**************************
 * UGV SOCKET
 **************************/
ugv_io.on('connect', (socket) => {
  console.log("UGV controls connected!");
  socket.emit('SET_TARGET', {lat: 38.14617, lng: -76.42642}); // Official competition destination

  socket.on('UGV_MESSAGE', (msg) => {
    if (config.verbose) console.log(msg);
    ui_io.emit('UGV_MESSAGE', msg);

    /*if (msg.status && msg.status.is_still != null) {
      let currentTime = Date.now();
      if (currentTime - ugvUnlatchTime > ugvWaitAfterUnlatchTime) {
        if (dropping && !ugvIsStill && msg.status.is_still) {
          ugvIsStill = true;
          ugvBecameStillTime = currentTime;
        } else if (dropping && msg.status.is_still && (currentTime - ugvBecameStillTime > ugvStillTimeThreshold)) {
          console.log("I think I should cut the fishing line!");
          // controls_io.emit('CHANGE_DROPPY_STATE', 'CUT_LINE'); // TODO should we actually do
        } else if (dropping && ugvIsStill && !msg.status.is_still) {
          ugvIsStill = false;
        }
      }
    }*/
  });

  socket.on('disconnect', () => {
    console.log("UGV controls disconnected");
  });
});


/**************************
 * BUTTON PANEL SOCKET
 **************************/
button_panel_io.on('connect', (socket) => {
  console.log("button panel connected!");
  
  socket.on('CHANGE_DROPPY_STATE', (state) => {
    controls_io.emit('CHANGE_DROPPY_STATE', state);
  });

  socket.on('disconnect', () => {
    console.log("button panel disconnected");
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

  socket.on('UPLOAD_IMAGE', (data) => {
    if (interopClient && interopData) {
      let odlc = {
        "mission": interopData.mission.id,
        "type": "STANDARD",
        "latitude": data.latitude ? data.latitude : 0,
        "longitude": data.longitude ? data.longitude : 0,
        "orientation": data.orient,
        "shape": data.shape,
        "shapeColor": data.shapeCol,
        "alphanumeric": data.letter,
        "alphanumericColor": data.letterCol,
        "autonomous": false
      }

      interopClient.postObjectDetails(odlc).then(returnedOdlc => {
        console.log("Submitted ODLC with id " + returnedOdlc.id);
        //TODO maybe imageFile will be a URL instead of a file path?
        interopClient.postObjectImage(data.imageFile, returnedOdlc.id).then(msg => {
          console.log(msg);
        }).catch(error => {
          console.log(error);
        });
      }).catch(error => {
        console.log(error);
      });
    }
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
    'HOTWIRE_SETPOINT',
    'CHANGE_DROPPY_STATE'
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
    // tracky_io.emit('CONFIGURE_POS', pos);
  });

  socket.on('SET_UGV_TARGET', (pos) => {
    console.log("Sending the UGV its target position");
    ugv_io.emit('SET_TARGET', pos);
  });

  socket.on('DRIVE_UGV', () => {
    console.log("Driving the UGV!");
    ugv_io.emit('DRIVE_TO_TARGET');
  });

  socket.on('DISABLE_UGV', () => {
    console.log("Disabling the UGV");
    ugv_io.emit('DISABLE');
  });

  socket.on('disconnect', () => {
    console.log("ui disconnected");
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
          if (config.verbose) console.log(droneIP + ": Not alive");
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

  socket.on('disconnect', () => {
    console.log("fake drone disconnected");
  });
});


if (config.testing && config.useFakeDrone) {
  require('./src/fake_drone/fake_drone');
}
