// import dotProp from 'dot-prop-immutable';

const initialState = {
  data: null,
  playback: false
};

export default function reducer(state=initialState, action) {
  switch (action.type) {
    case 'TELEMETRY': {
      if (!state.playback) {
        return {data: action.payload, playback: state.playback};
      } else {
        return state;
      }
    }
    case 'TOGGLE_PLAYBACK': {
      return {data: state.data, playback: !state.playback};
    }
    case 'PLAYBACK': {
      return action.payload;
    }
    default: {
      return state;
    }
  }
}
