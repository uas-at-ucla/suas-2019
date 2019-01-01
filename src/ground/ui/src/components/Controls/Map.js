import React, { Component } from 'react';
import { Marker } from 'react-google-maps';


import { connect } from 'react-redux';

import GoogleMap from '../Utils/GoogleMap/GoogleMap';

import reducersAndSelectors from  "../../reducers/missionReducer";
// const mapStateToProps = state => {
//   return {
//     getMarkers: reducersAndSelectors.selector.commandPoints
//   };
// };

class Map extends Component {
  render() {
    return (
      <div className="Map">
        <GoogleMap
          defaultZoom={16}
          defaultCenter={{ lat: 38.147483, lng: -76.427778 }}
          defaultMapTypeId="satellite"
          defaultOptions={{
            disableDefaultUI: true,
            disableDoubleClickZoom: true,
            scaleControl: true
          }}
        >
          {console.log(this.props)}
          <Marker position={{ lat: -34.397, lng: 150.644 }} />
        </GoogleMap>
      </div>
    );
  }
}

export default connect(reducersAndSelectors.selector)(Map);
