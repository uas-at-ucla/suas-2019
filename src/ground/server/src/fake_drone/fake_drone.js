const socketIOClient = require('socket.io-client');
const config = require('../../config');

const port = 8081;

const socket = socketIOClient('http://localhost:'+port+'/fake-drone', { transports: ['websocket'] });

const telemetry = require('./test_telemetry.json');

const sleepTime = 1000 / 50; // 50 Hz
let i = 0;
setInterval(() => {
  socket.emit('TELEMETRY', telemetry[i]);
  i = (i+1) % telemetry.length;
}, sleepTime);