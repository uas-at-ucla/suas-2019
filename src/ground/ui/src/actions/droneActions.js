//TAKEOFF", "LAND", "FAILSAFE", "THROTTLE_CUT"
const DroneStates = {
  TAKEOFF: 'TAKEOFF',
  LAND: 'LAND',
  FAILSAFE: 'FAILSAFE',
  THROTTLE_CUT: 'THROTTLE_CUT',
  RUN_MISSION: 'RUN_MISSION',
  HOLD: 'HOLD',
  OFFBOARD: 'OFFBOARD',
  RTL: 'RTL',
  LAND: 'LAND',
  ARM: 'ARM',
  DISARM: 'DISARM',
  ALARM: 'ALARM',
  BOMB_DROP: 'BOMB_DROP',
  DSLR: 'DSLR'
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
  droneRunMission: () => changeDroneState(DroneStates.RUN_MISSION),
  droneHold: () => changeDroneState(DroneStates.HOLD),
  droneOffboard: () => changeDroneState(DroneStates.OFFBOARD),
  droneRTL: () => changeDroneState(DroneStates.RTL),
  droneLand: () => changeDroneState(DroneStates.LAND),
  droneArm: () => changeDroneState(DroneStates.ARM),
  droneDisarm: () => changeDroneState(DroneStates.DISARM),
  droneAlarm: () => changeDroneState(DroneStates.ALARM),
  droneBombDrop: () => changeDroneState(DroneStates.BOMB_DROP),
  droneDSLR: () => changeDroneState(DroneStates.DSLR),
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
