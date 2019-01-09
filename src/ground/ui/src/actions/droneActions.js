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
