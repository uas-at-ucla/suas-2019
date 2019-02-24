import React, { Component } from 'react';
import CustomMarker from './CustomMarker';
import { Marker } from 'react-google-maps';

import GoogleMap from '../../Utils/GoogleMap/GoogleMap';

import './Map.css';

class Map extends Component {
    constructor() {
        super();

        this.state = {
            markers: [
                {
                    position: {
                        lat: -34.397,
                        lng: 150.644,
                    },
                    marker_i: 0,
                    data: 'first'
                },
                {
                    position: {
                        lat: -38,
                        lng: 150.644,
                    },
                    marker_i: 1,
                    data: 'second'
                }
            ],
        }
    }

    render() {
        let customMarkers;
        if (this.state.markers) {
            customMarkers = this.state.markers.map(marker => {
                console.log(marker.position)
                return (
                    <CustomMarker marker={marker} />
                )
            })
        }
        console.log(customMarkers);
        return (
            <div id="visionMap" className="Map">

                <GoogleMap
                    defaultZoom={8}
                    defaultCenter={{ lat: -34.397, lng: 150.644 }}
                    defaultMapTypeId="satellite"
                    defaultOptions={{
                        disableDefaultUI: true,
                        disableDoubleClickZoom: true,
                        scaleControl: true
                    }}
                >
                    {customMarkers}
                </GoogleMap>
            </div>
        );
    }
}

export default Map;
