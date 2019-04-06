import { createSelector, createStructuredSelector } from 'reselect';

const selectors = {};

selectors.droneMarker = createSelector(
  [state => state.telemetry.data],
  (telemetry) => {
    if (telemetry && telemetry.telemetry.sensors) {
      return {
        position: {
          lat: telemetry.telemetry.sensors.latitude,
          lng: telemetry.telemetry.sensors.longitude
        }
        // TODO make it an icon, rotate with drone heading, and show location uncertainty as a semi-transparent circle (HDOP)
      }
    } else {
      return null;
    }
  }
);

export default createStructuredSelector(selectors);
