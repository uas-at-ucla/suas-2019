// import dotProp from 'dot-prop-immutable';

const initialState = null;

export default function reducer(state=initialState, action) {
  switch (action.type) {
    case 'TELEMETRY': {
      return action.payload;
    }
    default: {
      return state;
    }
  }
}
