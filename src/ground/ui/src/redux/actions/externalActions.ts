export const serverConnected = () => ({
  type: "GND_SERVER_CONNECTED" as const
});

export const serverDisconnected = () => ({
  type: "GND_SERVER_DISCONNECTED" as const
});

const basicMessages = [
  "TELEMETRY",
  "COMPILED_DRONE_PROGRAM",
  "UPLOADED_DRONE_PROGRAM",
  "MISSION_STATUS",
  "GIMBAL_SETPOINT",
  "DEPLOYMENT_MOTOR_SETPOINT",
  "LATCH_SETPOINT",
  "HOTWIRE_SETPOINT",
  "INTEROP_DATA",
  "PING",
  "UGV_MESSAGE",
  "DROPPY_COMMAND_RECEIVED"
] as const;

export const basicServerAction = (
  msg: typeof basicMessages[number],
  data: any
) => ({
  type: msg,
  payload: data
});
basicServerAction.basicMessages = basicMessages;

export const timelineProtoLoaded = (timelineProto: any) => ({
  type: "TIMELINE_PROTO_LOADED" as const,
  payload: timelineProto
});
