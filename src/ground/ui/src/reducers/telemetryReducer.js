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
    droneMarker: (state) => {
      if (state && state.telemetry.sensors) {
        return {
          position: {
            lat: state.telemetry.sensors.latitude,
            lng: state.telemetry.sensors.longitude
          }
          // TODO make it an icon, rotate with drone heading, and show location uncertainty as a semi-transparent circle (HDOP)
        }
      } else {
        return null;
      }
    }
  })
}
