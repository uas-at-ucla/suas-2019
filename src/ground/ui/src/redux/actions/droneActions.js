//TAKEOFF", "LAND", "FAILSAFE", "THROTTLE_CUT"
const DroneStates = {
  TAKEOFF: 'TAKEOFF',
  LAND: 'LAND',
  FAILSAFE: 'FAILSAFE',
  THROTTLE_CUT: 'THROTTLE_CUT'
}
function changeDroneState(droneState) {
  return {
    type: 'TRANSMIT',
    payload: {
      msg: 'CHANGE_DRONE_STATE',
      data: droneState
    }
  }
}

const DroppyStates = {
  START_DROP: 'START_DROP',
  CUT_LINE: 'CUT_LINE',
  MOTOR_UP: 'MOTOR_UP',
  MOTOR_DOWN: 'MOTOR_DOWN',
  MOTOR_STOP: 'MOTOR_STOP',
  CANCEL_DROP: 'CANCEL_DROP',
  RESET_LATCH: 'RESET_LATCH',
  STOP_CUT: 'STOP_CUT'

}
function changeDroppyState(droppyState) {
  return {
    type: 'TRANSMIT',
    payload: {
      msg: 'CHANGE_DROPPY_STATE',
      data: droppyState
    }
  }
}

export default {
  droneTakeoff: () => changeDroneState(DroneStates.TAKEOFF),
  droneLand: () => changeDroneState(DroneStates.LAND),
  droneFailsafe: () => changeDroneState(DroneStates.FAILSAFE),
  droneThrottleCut: () => changeDroneState(DroneStates.THROTTLE_CUT),
  droppyStart: () => changeDroppyState(DroppyStates.START_DROP),
  droppyCut: () => changeDroppyState(DroppyStates.CUT_LINE),
  droppyUp: () => changeDroppyState(DroppyStates.MOTOR_UP),
  droppyDown: () => changeDroppyState(DroppyStates.MOTOR_DOWN),
  droppyStop: () => changeDroppyState(DroppyStates.MOTOR_STOP),
  droppyCancel: () => changeDroppyState(DroppyStates.CANCEL_DROP),
  droppyResetLatch: () => changeDroppyState(DroppyStates.RESET_LATCH),
  droppyStopCut: () => changeDroppyState(DroppyStates.STOP_CUT),
  compileMission: (commands) => {
    return {
      type: 'TRANSMIT',
      payload: {
        msg: 'COMPILE_GROUND_PROGRAM',
        data: commands
      }
    }
  },
  uploadMission: () => {
    return {
      type: 'TRANSMIT',
      payload: {msg: 'UPLOAD_MISSION'}
    }
  },
  runMission: () => {
    return {
      type: 'TRANSMIT',
      payload: {msg: 'RUN_MISSION'}
    }
  },
  pauseMission: () => {
    return {
      type: 'TRANSMIT',
      payload: {msg: 'PAUSE_MISSION'}
    }
  },
  endMission: () => {
    return {
      type: 'TRANSMIT',
      payload: {msg: 'END_MISSION'}
    }
  },
  driveUgv: () => {
    return {
      type: 'TRANSMIT',
      payload: { msg: 'DRIVE_UGV' }
    }
  },
  disableUgv: () => {
    return {
      type: 'TRANSMIT',
      payload: { msg: 'DISABLE_UGV' }
    }
  },
  sendSetpoint: (actuator, value) => {
    return {
      type: 'TRANSMIT',
      payload: { msg: actuator, data: value }
    }
  },
  testTransmit: (data) => {
    return {
      type: 'TRANSMIT',
      payload: {
        msg: 'TEST',
        data: data
      }
    }
  }
}
