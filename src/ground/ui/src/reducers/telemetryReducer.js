// import dotProp from 'dot-prop-immutable';
import { createSelector, createStructuredSelector } from 'reselect';

const initialState = null;

export default {
  reducer: (state=initialState, action) => {
    switch (action.type) {
      case 'TELEMETRY': {
        return action.payload;
        // dot-prop-immutable example: return dotProp.set(state, `property1.property2`, "some value");
      }
      default: {
        return state;
      }
    }
  },

  selector: createStructuredSelector({
    droneLatLng: (state) => {
      if (state && state.telemetry.sensors) {
        return {
          lat: state.telemetry.sensors.latitude,
          lng: state.telemetry.sensors.longitude
        }
      } else {
        return null;
      }
    }
  })
}
