import { fromJS } from 'immutable'

const initialState = fromJS({
  value: 0
});

export default function reducer(state=initialState, action) {
  switch (action.type) {
    case 'ADD': {
      return state.update('value', val => val + action.payload);
    }
    default: {
      return state;
    }
  }
}
