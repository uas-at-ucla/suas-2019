import socketIOClient from 'socket.io-client';

const isWebServer = window.location.protocol.startsWith("http");

const serverIP = "localhost"; // TODO: Make configurable via Settings
const socketHost = isWebServer ? window.location.hostname : serverIP;
const socketPort = 8081;
const socket = socketIOClient("http://"+socketHost+':'+socketPort+'/ui', { transports: ['websocket'] });

export default (store) => {
  console.log("Initializing communicator");

  socket.on('TELEMETRY', (data) => {
    store.dispatch({ type: 'TELEMETRY', payload: data });
  });

  socket.on('INTEROP_DATA', (data) => {
    store.dispatch({ type: 'INTEROP_DATA', payload: data });
  });

  return (next) => (action) => {
    if (action.type === 'TRANSMIT') {
      socket.emit(action.payload.msg, action.payload.data);
      console.log("Transmitting", action.payload);
    }
    next(action);
  }
}