import { SettingsState } from "redux/reducers/settingsReducer";
import { transmit } from "./genericActions";

export const updateSettings = (newSettings: SettingsState) => ({
  type: "UPDATE_SETTINGS" as const,
  payload: newSettings
});

export const connectToInterop = (
  ip: string,
  username: string,
  password: string,
  missionId: number
) =>
  transmit("CONNECT_TO_INTEROP", {
    ip: ip,
    username: username,
    password: password,
    missionId: missionId
  });

export const connectToGndServer = () => ({
  type: "CONNECT_TO_GND_SERVER" as const
});

export const configureTrackyPos = (pos: any) =>
  transmit("CONFIGURE_TRACKY_POS", pos);

export const configureUgvDest = (pos: any) => transmit("SET_UGV_TARGET", pos);
