const socketIOClient = require('socket.io-client');
const constants = require('../config');

const port = 8081;

const socket = socketIOClient('http://localhost:'+port+'/fake-drone', { transports: ['websocket'] });

const telemetry = require('./test_telemetry.json').list;

const sleepTime = 1000 / constants.droneTelemetryFrequency;
let i = 0;
setInterval(() => {
  socket.emit('TELEMETRY', telemetry[i]);
  i = (i+1) % telemetry.length;
}, sleepTime);