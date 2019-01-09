import dotProp from 'dot-prop-immutable';

const initialState = null;

export default function reducer(state=initialState, action) {
  switch (action.type) {
    case 'TELEMETRY': {
      return action.payload;
      // dot-prop-immutable example: return dotProp.set(state, `property1.property2`, "some value");
    }
    default: {
      return state;
    }
  }
}
