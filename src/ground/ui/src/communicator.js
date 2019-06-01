import socketIOClient from 'socket.io-client';

class Communicator {
  constructor(store) {
    this.store = store;
    console.log("Initializing communicator");
    this.initSocket();
  }

  initSocket() {
    this.socket = socketIOClient("http://"+this.store.getState().settings.gndServerIp+'/ui', { transports: ['websocket'] });
    
    this.socket.on('connect', () => {
      this.store.dispatch({ type: 'GND_SERVER_CONNECTED' });
    });

    this.socket.on('disconnect', () => {
      this.store.dispatch({ type: 'GND_SERVER_DISCONNECTED' });
    });

    this.socket.on('TELEMETRY', (telemetry) => {
      this.store.dispatch({ type: 'TELEMETRY', payload: telemetry });
    });

    this.socket.on('COMPILED_DRONE_PROGRAM', (droneProgram) => {
      this.store.dispatch({ type: 'COMPILED_DRONE_PROGRAM', payload: droneProgram });
    });
    
    this.socket.on('MISSION_COMPILE_ERROR', () => {
      alert("FAILED to compile mission!");
    });

    this.socket.on('UPLOADED_DRONE_PROGRAM', (droneProgram) => {
      this.store.dispatch({ type: 'UPLOADED_DRONE_PROGRAM', payload: droneProgram });
    });

    this.socket.on('MISSION_STATUS', (status) => {
      this.store.dispatch({ type: 'MISSION_STATUS', payload: status });
    });

    this.socket.on('INTEROP_DATA', (interopData) => {
      this.store.dispatch({ type: 'INTEROP_DATA', payload: interopData });
    });

    this.socket.on('PING', (delay) => {
      this.store.dispatch({ type: 'PING', payload: delay });
    });

    this.socket.on('INTEROP_UPLOAD_FAIL', () => {
      alert("FAILED to upload telemetry to interop!");
    });

    this.socket.on('INTEROP_UPLOAD_SUCCESS', () => {
      alert("Now able to upload telemetry to interop. :)");
    });

    this.socket.on('UGV_MESSAGE', (msg) => {
      this.store.dispatch({ type: 'UGV_MESSAGE', payload: msg });
    });
  }

  reduxMiddleware(next) {
    return (action) => {
      if (action.type === 'TRANSMIT') {
        if (action.payload.data) {
          this.socket.emit(action.payload.msg, action.payload.data);
        } else {
          this.socket.emit(action.payload.msg);
        }
        console.log("Transmitting", action.payload);
      } else if (action.type === 'CONNECT_TO_GND_SERVER') {
        this.socket.disconnect();
        this.initSocket();
      }
      next(action);
    }
  }
}

export default (store) => {
  let communicator = new Communicator(store);
  return (next) => communicator.reduxMiddleware(next);
}