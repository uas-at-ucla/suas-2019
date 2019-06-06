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

    let basic_messages = [
      'TELEMETRY',
      'COMPILED_DRONE_PROGRAM',
      'UPLOADED_DRONE_PROGRAM',
      'MISSION_STATUS',
      'GIMBAL_SETPOINT',
      'DEPLOYMENT_MOTOR_SETPOINT',
      'LATCH_SETPOINT',
      'HOTWIRE_SETPOINT',
      'INTEROP_DATA',
      'PING',
      'UGV_MESSAGE',
      'DROPPY_COMMAND_RECEIVED'
    ];
    for (let message of basic_messages) {
      this.socket.on(message, (data) => {
        this.store.dispatch({ type: message, payload: data });
      });
    }
    
    this.socket.on('MISSION_COMPILE_ERROR', () => {
      alert("FAILED to compile mission!");
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
        if (action.payload.data != null) {
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