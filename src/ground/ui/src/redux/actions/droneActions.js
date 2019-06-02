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

export default {
  droneTakeoff: () => changeDroneState(DroneStates.TAKEOFF),
  droneLand: () => changeDroneState(DroneStates.LAND),
  droneFailsafe: () => changeDroneState(DroneStates.FAILSAFE),
  droneThrottleCut: () => changeDroneState(DroneStates.THROTTLE_CUT),
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
