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

    this.socket.on('TELEMETRY', (data) => {
      this.store.dispatch({ type: 'TELEMETRY', payload: data });
    });

    this.socket.on('INTEROP_DATA', (data) => {
      this.store.dispatch({ type: 'INTEROP_DATA', payload: data });
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