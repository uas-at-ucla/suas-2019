import socketIOClient from "socket.io-client";
import { Dispatch, Middleware, MiddlewareAPI } from "redux";
import { AppAction } from "redux/actions/actionTypes";
import * as externalActions from "redux/actions/externalActions";

class Communicator {
  private store: MiddlewareAPI;
  private socket: SocketIOClient.Socket;

  public constructor(store: MiddlewareAPI) {
    console.log("Initializing communicator");
    this.store = store;
    this.socket = this.initSocket();
  }

  private initSocket(): SocketIOClient.Socket {
    this.socket = socketIOClient(
      "http://" + this.store.getState().settings.gndServerIp + "/ui",
      { transports: ["websocket"] }
    );

    this.socket.on("connect", (): void => {
      this.store.dispatch(externalActions.serverConnected());
    });

    this.socket.on("disconnect", (): void => {
      this.store.dispatch(externalActions.serverDisconnected());
    });

    for (let message of externalActions.basicServerAction.basicMessages) {
      this.socket.on(message, (data: any) => {
        this.store.dispatch(externalActions.basicServerAction(message, data));
      });
    }

    this.socket.on("MISSION_COMPILE_ERROR", () => {
      alert("FAILED to compile mission!");
    });

    this.socket.on("INTEROP_UPLOAD_FAIL", () => {
      alert("FAILED to upload telemetry to interop!");
    });

    this.socket.on("INTEROP_UPLOAD_SUCCESS", () => {
      alert("Now able to upload telemetry to interop. :)");
    });

    return this.socket;
  }

  public reduxMiddleware(next: Dispatch) {
    return (action: AppAction) => {
      if (action.type === "TRANSMIT") {
        if (action.payload.data != null) {
          this.socket.emit(action.payload.msg, action.payload.data);
        } else {
          this.socket.emit(action.payload.msg);
        }
        console.log("Transmitting", action.payload);
      } else if (action.type === "CONNECT_TO_GND_SERVER") {
        this.socket.disconnect();
        this.initSocket();
      }
      next(action);
    };
  }
}

export default ((store: MiddlewareAPI) => {
  let communicator = new Communicator(store);
  return (next: Dispatch) => communicator.reduxMiddleware(next);
}) as Middleware;
