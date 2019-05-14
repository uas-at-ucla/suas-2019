import { createSelector, createStructuredSelector } from 'reselect';

const selectors = {};

selectors.droneMarker = createSelector(
  [state => state.telemetry.droneTelemetry],
  (droneTelemetry) => {
    if (droneTelemetry && droneTelemetry.sensors) {
      return {
        position: {
          lat: droneTelemetry.sensors.latitude,
          lng: droneTelemetry.sensors.longitude
        },
        icon: {
          path: window.google ? window.google.maps.SymbolPath.FORWARD_CLOSED_ARROW : null,
          strokeColor: '#FFFFFF',
          strokeOpacity: 0.8,
          strokeWeight: 3,
          fillColor: '#0000FF',
          fillOpacity: 0.5,
          scale: 7,
          rotation: droneTelemetry.sensors.heading,
          anchor: window.google ? new window.google.maps.Point(0, 2.5) : null
        },
        eph: droneTelemetry.sensors.gpsEph
        // TODO make it an icon, rotate with drone heading, and show location uncertainty as a semi-transparent circle (HDOP)
      }
    } else {
      return null;
    }
  }
);

export default createStructuredSelector(selectors);
