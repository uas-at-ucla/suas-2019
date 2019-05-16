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
  }

  reduxMiddleware(next) {
    return (action) => {
      if (action.type === 'TRANSMIT') {
        this.socket.emit(action.payload.msg, action.payload.data);
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