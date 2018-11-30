import React, { Component } from 'react';
import CustomMarker from './CustomMarker';
import { Marker } from 'react-google-maps';
import Modal from 'react-modal';

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
                    marker_n: 0,
                    data: 'first'
                },
                {
                    position: {
                        lat: -38,
                        lng: 150.644,
                    },
                    marker_n: 1,
                    data: 'second'
                }
            ],
            modalIsOpen: false,
            modalData: 'hello'
        }

        this.openModal = this.openModal.bind(this);
        this.closeModal = this.closeModal.bind(this);
    }

    openModal(data) {
        this.setState({ modalData: data, modalIsOpen: true });
    }

    closeModal() {
        this.setState({ modalIsOpen: false });
    }

    render() {
        let customMarkers;
        if (this.state.markers) {
            customMarkers = this.state.markers.map(marker => {
                console.log(marker.position)
                return (
                    <CustomMarker marker={marker} openModal={this.openModal} />
                )
            })
        }
        console.log(customMarkers);
        return (
            <div id="visionMap" className="Map">
                <Modal id = "mymodal"
                    isOpen={this.state.modalIsOpen}
                    onRequestClose={this.closeModal}
                >
                    <h2 ref={subtitle => this.subtitle = subtitle}>Hello</h2>
                    <div>click outside to exit modal</div>
                    <div>{this.state.modalData}</div>
                </Modal>

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