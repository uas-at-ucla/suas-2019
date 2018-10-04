import { fromJS } from 'immutable'

const initialState = null;

export default function reducer(state=initialState, action) {
  switch (action.type) {
    case 'TELEMETRY': {
      return fromJS(action.payload);
    }
    default: {
      return state;
    }
  }
}
