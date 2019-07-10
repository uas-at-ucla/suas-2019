import { AppAction } from "../actions/actionTypes";

export interface SettingsState {
  gndServerIp: string;
  connectedGndServerIp: string;
  gndServerConnected: boolean;
  interopIp: string;
  interopUsername: string;
  interopPassword: string;
  interopMissionId: number;
  antennaPos: { lat: number; lng: number };
}

const isWebServer = window.location.protocol.startsWith("http");
const defaultIP = "localhost";
const socketHost = isWebServer ? window.location.hostname : defaultIP;
const socketPort = 8081;

const initialState: SettingsState = {
  gndServerIp: socketHost + ":" + socketPort,
  connectedGndServerIp: socketHost + ":" + socketPort,
  gndServerConnected: false,
  interopIp: "134.209.2.203:8000",
  interopUsername: "testuser",
  interopPassword: "testpass",
  interopMissionId: 2,
  antennaPos: { lat: 0, lng: 0 }
};

export default function reducer(
  state = initialState,
  action: AppAction
): SettingsState {
  switch (action.type) {
    case "RESET_REDUX_STATE": {
      return initialState;
    }
    case "UPDATE_SETTINGS": {
      return { ...state, ...action.payload };
    }
    case "CONNECT_TO_GND_SERVER": {
      return { ...state, connectedGndServerIp: state.gndServerIp };
    }
    case "GND_SERVER_CONNECTED": {
      return { ...state, gndServerConnected: true };
    }
    case "GND_SERVER_DISCONNECTED": {
      return { ...state, gndServerConnected: false };
    }
    default: {
      return state;
    }
  }
}
