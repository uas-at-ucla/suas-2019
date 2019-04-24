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
        },
        icon: {
          path: window.google ? window.google.maps.SymbolPath.FORWARD_CLOSED_ARROW : null,
          strokeColor: '#FFFFFF',
          strokeOpacity: 0.8,
          strokeWeight: 3,
          fillColor: '#0000FF',
          fillOpacity: 0.5,
          scale: 7,
          rotation: telemetry.telemetry.sensors.heading,
          anchor: window.google ? new window.google.maps.Point(0, 2.5) : null
        },
        eph: telemetry.telemetry.sensors.gpsEph
        // TODO make it an icon, rotate with drone heading, and show location uncertainty as a semi-transparent circle (HDOP)
      }
    } else {
      return null;
    }
  }
);

export default createStructuredSelector(selectors);
