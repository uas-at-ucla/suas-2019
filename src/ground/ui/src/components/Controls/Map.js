import React, { Component } from 'react';
import { Marker, InfoWindow } from 'react-google-maps';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';

import GoogleMap from '../Utils/GoogleMap/GoogleMap';
import missionActions from '../../actions/missionActions';
import { selector } from '../../store';
import { Circle } from "react-google-maps";

const mapStateToProps = state => {
  let derivedData = selector(state);
  return {
    commandPoints: derivedData.missionPlan.commandPoints,
    protoInfo: derivedData.missionPlan.protoInfo,
    telemetry: state.telemetry,
    droneMarker: derivedData.telemetry.droneMarker 
  };
};

const mapDispatchToProps = missionActions;

class Map extends Component {
  state = {
    isOpen: {},
    mission:
    [ 
        {
            "id": 1,
            "active": true,
            "air_drop_pos": {
                "latitude": 38.141833,
                "longitude": -76.425263
            },
            "fly_zones": [
                {
                    "altitude_msl_max": 200.0,
                    "altitude_msl_min": 100.0,
                    "boundary_pts": [
                        {
                            "latitude": 38.142544,
                            "longitude": -76.434088,
                            "order": 1
                        },
                        {
                            "latitude": 38.141833,
                            "longitude": -76.425263,
                            "order": 2
                        },
                        {
                            "latitude": 38.144678,
                            "longitude": -76.427995,
                            "order": 3
                        }
                    ]
                }
            ],
            "home_pos": {
                "latitude": 38.14792,
                "longitude": -76.427995
            },
            "mission_waypoints": [
                {
                    "altitude_msl": 200.0,
                    "latitude": 38.142544,
                    "longitude": -76.434088,
                    "order": 1
                }
            ],
            "off_axis_odlc_pos": {
                "latitude": 38.142544,
                "longitude": -76.434088
            },
            "emergent_last_known_pos": {
                "latitude": 38.145823,
                "longitude": -76.422396
            },
            "search_grid_points": [
                {
                    "altitude_msl": 200.0,
                    "latitude": 38.142544,
                    "longitude": -76.434088,
                    "order": 1
                }
            ]
        }
      ],
    stationary_obstacles : [
          {
              "cylinder_height": 750.0,
              "cylinder_radius": 300.0,
              "latitude": 38.140578,
              "longitude": -76.428997
          },
          {
              "cylinder_height": 400.0,
              "cylinder_radius": 100.0,
              "latitude": 38.149156,
              "longitude": -76.430622
          }
        ]
  };


  onToggleOpen = (id) => {
    this.setState({
      isOpen: {...this.state.isOpen, [id]: !this.state.isOpen[id]}
    });
  }
 
  onMapClick = () => {
    if (this.state.isOpen) {
      this.setState({
        isOpen: {}
      })
    }
  };
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
          onClick ={this.onMapClick}
          onDblClick={this.mapDblClick}

        >
          {this.props.droneMarker ? 
            <Marker {...this.props.droneMarker}></Marker> 
          : null}

            <Marker  
            title="airDropPosition"
            position={{lat: this.state.mission[0].air_drop_pos.latitude, lng: this.state.mission[0].air_drop_pos.longitude}}
            onClick = {()=>this.onToggleOpen("air_drop_pos")}>          
            {this.state.isOpen["air_drop_pos"] && <InfoWindow onCloseClick = {() =>this.onToggleOpen("air_drop_pos")}>
           <div className="map-infobox">Air Drop Position</div>
            </InfoWindow>}
            </Marker>
          
            <Marker  
            title="homePosition"
            position={{lat: this.state.mission[0].home_pos.latitude, lng: this.state.mission[0].home_pos.longitude}}
            onClick = {()=>this.onToggleOpen("home_pos")}>        
            {this.state.isOpen["home_pos"] && <InfoWindow onCloseClick = {() =>this.onToggleOpen("home_pos")}>
            <div className="map-infobox">Home Position</div>
            </InfoWindow>}          
            </Marker>


            <Circle
              radius={this.state.stationary_obstacles[0].cylinder_radius}
              center={{lat: this.state.stationary_obstacles[0].latitude, lng: this.state.stationary_obstacles[0].longitude}}
          />

            <Circle 
            radius={this.state.stationary_obstacles[1].cylinder_radius}
            center={{lat: this.state.stationary_obstacles[1].latitude, lng: this.state.stationary_obstacles[1].longitude}}
            ></Circle>
    

          {this.props.commandPoints.map((commandPoint, index) => 
            commandPoint ?
              <Marker {...commandPoint.marker} key={commandPoint.id} onClick = {()=>this.onToggleOpen(commandPoint.id)}>
                {this.state.isOpen[commandPoint.id] && <InfoWindow {...commandPoint.infobox} onCloseClick = {() =>this.onToggleOpen(commandPoint.id)}>
                  <div className="map-infobox">
                    <div>
                      {/* TODO: add title */}
                      {commandPoint.infobox.title}
                    </div>
                    <div>
                     {commandPoint.infobox.content}
                    </div>
                   
                    <Button onClick={this.deleteCommand} data-index={index}>
                      Delete Command
                    </Button>
                  </div>
                </InfoWindow>}
              </Marker>
            : null
          )}

        </GoogleMap>
      </div>
    );
  }

  deleteCommand = (event) => {
    this.props.deleteCommand(event.target.dataset.index);
  }

  mapDblClick = (event) => {
    this.addWaypointCommand(event.latLng.lat(), event.latLng.lng());
  }

  addWaypointCommand = (lat, lng) => {
    let defaultWaypointCommand = { goal: {
      latitude: lat,
      longitude: lng,
      altitude: 100
    }}
    this.props.addWaypointCommand(defaultWaypointCommand, this.props.protoInfo);
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(Map);
