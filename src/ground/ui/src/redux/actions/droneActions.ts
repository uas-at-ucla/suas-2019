import { transmit } from "./genericActions";

const DroneStates = {
  TAKEOFF: "TAKEOFF",
  LAND: "LAND",
  FAILSAFE: "FAILSAFE",
  THROTTLE_CUT: "THROTTLE_CUT"
};

const changeDroneState = (droneState: string) =>
  transmit("CHANGE_DRONE_STATE", droneState);

const DroppyStates = {
  START_DROP: "START_DROP",
  CUT_LINE: "CUT_LINE",
  MOTOR_UP: "MOTOR_UP",
  MOTOR_DOWN: "MOTOR_DOWN",
  MOTOR_STOP: "MOTOR_STOP",
  CANCEL_DROP: "CANCEL_DROP",
  RESET_LATCH: "RESET_LATCH",
  STOP_CUT: "STOP_CUT"
};

const changeDroppyState = (droppyState: string) =>
  transmit("CHANGE_DROPPY_STATE", droppyState);

export const droneTakeoff = () => changeDroneState(DroneStates.TAKEOFF);
export const droneLand = () => changeDroneState(DroneStates.LAND);
export const droneFailsafe = () => changeDroneState(DroneStates.FAILSAFE);
export const droneThrottleCut = () =>
  changeDroneState(DroneStates.THROTTLE_CUT);

export const droppyStart = () => changeDroppyState(DroppyStates.START_DROP);
export const droppyCut = () => changeDroppyState(DroppyStates.CUT_LINE);
export const droppyUp = () => changeDroppyState(DroppyStates.MOTOR_UP);
export const droppyDown = () => changeDroppyState(DroppyStates.MOTOR_DOWN);
export const droppyStop = () => changeDroppyState(DroppyStates.MOTOR_STOP);
export const droppyCancel = () => changeDroppyState(DroppyStates.CANCEL_DROP);
export const droppyResetLatch = () =>
  changeDroppyState(DroppyStates.RESET_LATCH);
export const droppyStopCut = () => changeDroppyState(DroppyStates.STOP_CUT);

export const compileMission = (commands: any[]) =>
  transmit("COMPILE_GROUND_PROGRAM", commands);
export const uploadMission = () => transmit("UPLOAD_MISSION");
export const runMission = () => transmit("RUN_MISSION");
export const pauseMission = () => transmit("PAUSE_MISSION");
export const endMission = () => transmit("END_MISSION");

export const driveUgv = () => transmit("DRIVE_UGV");
export const disableUgv = () => transmit("DISABLE_UGV");
export const sendSetpoint = (actuator: string, value: number | boolean) =>
  transmit(actuator, value);
