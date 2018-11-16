import React, { Component } from 'react';
import { Marker, Polyline } from 'react-google-maps';
import { Circle } from 'react-google-maps';

import GoogleMap from '../Utils/GoogleMap/GoogleMap';


class Map extends Component {
  state = {
    commands: [
      {
        latitude: 34.068528,
        longitude: -118.442990,
        type: "Waypoint",
        name: "Waypoint 1",
      },
      {
        latitude: 37.795601, 
        longitude: -121.964005,
        type: "Waypoint",
        name: "Waypoint 2",
      }
    ]
  }

  pathCoordinates = [
    { lat: this.state.commands[0].latitude, lng: this.state.commands[0].longitude },
    { lat: this.state.commands[1].latitude, lng: this.state.commands[1].longitude }
  ]

  render() {
    return (
      <div className="Map">
        <GoogleMap
          defaultZoom={6}
          defaultCenter={{ lat: this.state.commands[0].latitude, lng: this.state.commands[0].longitude }}
          defaultMapTypeId="satellite"
          defaultOptions={{
            disableDefaultUI: true,
            disableDoubleClickZoom: true,
            scaleControl: true
          }}
        >
          <Marker position={{ lat: this.state.commands[0].latitude, lng: this.state.commands[0].longitude }} />
          <Marker position={{ lat: this.state.commands[1].latitude, lng: this.state.commands[1].longitude }} />
          <Polyline path={this.pathCoordinates} 
                    options={{strokeColor: 'red', strokeWeight: 3}} 
                    visible={true} 
                    draggable={false}
                    
          />
          <Circle defaultCenter={this.pathCoordinates[0]}
                  radius={200000}
                  options ={{strokeColor: 'cyan', fillColor: 'blue'}}
                  draggable={true}
          />
        </GoogleMap>
      </div>
    );
  }
}

export default Map;
