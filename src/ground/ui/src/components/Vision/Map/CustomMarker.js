import React, { Component } from 'react';
import { Marker } from 'react-google-maps';

class CustomMarker extends Component {
    render() {
        console.log("creating marker");
        let latitude = this.props.marker.position.lat;
        let longitude = this.props.marker.position.lng;
        console.log({latitude, longitude});
        return (
            <Marker onClick={() => this.props.openModal(this.props.marker.data)} position={{ lat: latitude, lng: longitude }} />
        );
    }
}

export default CustomMarker;