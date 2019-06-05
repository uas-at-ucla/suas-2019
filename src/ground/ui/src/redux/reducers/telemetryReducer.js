import dotProp from 'dot-prop-immutable';

const initialState = {
  droneTelemetry: null,
  playback: false,
  recording: false,
  telemetryData: [],
  pingDelay: null,
  mapCenter: { lat: 38.147483, lng: -76.427778 },
  ugvStatus: null,
  setpoints: {
    gimbal: null,
    deployment: null,
    latch: null,
    hotwire: null,
  }
};

export default function reducer(state=initialState, action) {
  switch (action.type) {
    case 'PING': {
      return {...state, pingDelay: action.payload};
    }
    case 'TELEMETRY': {
      if (!state.playback) {
        let newState = {...state, droneTelemetry: action.payload};
        // if recording add to list
        if (state.recording) {
          newState.telemetryData.push(action.payload)
        }
        return newState;
      } else {
        return state;
      }
    }
    case 'TOGGLE_PLAYBACK': {
      return {...state, playback: !state.playback};
    }
    case 'TOGGLE_RECORD': {
      return {...state, recording: !state.recording}
    }
    case 'PLAYBACK': {
      return {...state, droneTelemetry: action.payload};
    }
    case 'GIMBAL_SETPOINT': {
      return dotProp.set(state, `setpoints.gimbal`, action.payload);
    }
    case 'DEPLOYMENT_MOTOR_SETPOINT': {
      return dotProp.set(state, `setpoints.deployment`, action.payload);
    }
    case 'LATCH_SETPOINT': {
      return dotProp.set(state, `setpoints.latch`, action.payload);
    }
    case 'HOTWIRE_SETPOINT': {
      return dotProp.set(state, `setpoints.hotwire`, action.payload);
    }
    case 'CENTER_ON_DRONE': {
      if (state.droneTelemetry) {
        return {
          ...state,
          mapCenter: {
            lat: state.droneTelemetry.sensors.latitude,
            lng: state.droneTelemetry.sensors.longitude
          }
        };
      } else {
        return state;
      }
    }
    case 'CENTER_ON_COMMAND': {
      return {...state, mapCenter: action.payload.pos};
    }
    case 'INTEROP_DATA': {
      if (!action.payload) {
        return state;
      }
      return {
        ...state,
        mapCenter: {
          lat: action.payload.mission.airDropPos.latitude,
          lng: action.payload.mission.airDropPos.longitude
        }
      };
    }
    case 'UGV_MESSAGE': {
      return {...state, ugvStatus: action.payload.status};
    }
    default: {
      return state;
    }
  }
}
