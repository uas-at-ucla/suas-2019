import React, { Component } from 'react';
import { Marker, InfoWindow } from 'react-google-maps';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';

import GoogleMap from '../Utils/GoogleMap/GoogleMap';
import missionActions from '../../actions/missionActions';
import { selector } from '../../store';
import { Circle, Polygon } from "react-google-maps";


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
    {
      "fly_zones": [
          {
              "altitude_msl_min": 0,
              "altitude_msl_max": 750,
              "boundary_pts": [
                  { "latitude": 38.14627, "longitude": -76.42816 },
                  { "latitude": 38.15162, "longitude": -76.42868 },
                  { "latitude": 38.15189, "longitude": -76.43147 },
                  { "latitude": 38.15059, "longitude": -76.43536 },
                  { "latitude": 38.14757, "longitude": -76.43234 },
                  { "latitude": 38.14467, "longitude": -76.43295 },
                  { "latitude": 38.14326, "longitude": -76.43477 },
                  { "latitude": 38.14046, "longitude": -76.43264 },
                  { "latitude": 38.14072, "longitude": -76.42601 },
                  { "latitude": 38.14376, "longitude": -76.42121 },
                  { "latitude": 38.14735, "longitude": -76.42321 },
                  { "latitude": 38.14613, "longitude": -76.42665 }
              ]
          }
      ],
      "search_grid_points": [
          { "latitude": 38.14576, "longitude": -76.42969 },
          { "latitude": 38.14323, "longitude": -76.43379 },
          { "latitude": 38.14123, "longitude": -76.43233 },
          { "latitude": 38.14139, "longitude": -76.42709 },
          { "latitude": 38.14221, "longitude": -76.42611 }
      ],
      "mission_waypoints": [
          { "latitude": 38.15079, "longitude": -76.43044, "altitude_msl": 150 },
          { "latitude": 38.14961, "longitude": -76.43295, "altitude_msl": 200 },
          { "latitude": 38.14218, "longitude": -76.42564, "altitude_msl": 200 },
          { "latitude": 38.14388, "longitude": -76.42263, "altitude_msl": 200 },
          { "latitude": 38.14564, "longitude": -76.42424, "altitude_msl": 200 },
          { "latitude": 38.14400, "longitude": -76.42875, "altitude_msl": 250 }
      ],
      "emergent_last_known_pos": { "latitude": 38.145762, "longitude": -76.423065 },
      "off_axis_odlc_pos": { "latitude": 38.147635, "longitude": -76.427249 },
      "home_pos": { "latitude": 38.145323, "longitude": -76.428000 },
      "air_drop_pos": { "latitude": 38.145830, "longitude": -76.426391 },
      "stationary_obstacles": [
          { "latitude": 38.15079, "longitude": -76.43044, "cylinder_radius": 100, "cylinder_height": 100 }
      ],
      "moving_obstacles": [
          { 
              "sphere_radius": 300,
              "speed_avg": 30,
              "waypoints": [
                  { "latitude": 38.15079, "longitude": -76.43044, "altitude_msl": 100},
                  { "latitude": 38.14139, "longitude": -76.42709, "altitude_msl": 150}
              ]
          }
      ]
    }
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
    var lineCoordinates =[];
    const boundaryCoordinates = this.state.mission.fly_zones[0].boundary_pts.map((coord, index) => {
      lineCoordinates[index] = {lat: coord.latitude, lng: coord.longitude };
      return lineCoordinates[index];
    })
    /*const lineCoordinates = [
      { lat: this.state.mission.fly_zones[0].boundary_pts[0].latitude ,lng: this.state.mission.fly_zones[0].boundary_pts[0].longitude},
      { lat: this.state.mission.fly_zones[0].boundary_pts[1].latitude ,lng: this.state.mission.fly_zones[0].boundary_pts[1].longitude},
      { lat: this.state.mission.fly_zones[0].boundary_pts[2].latitude ,lng: this.state.mission.fly_zones[0].boundary_pts[2].longitude}
       ];*/
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
            position={{lat: this.state.mission.air_drop_pos.latitude, lng: this.state.mission.air_drop_pos.longitude}}
            onClick = {()=>this.onToggleOpen("air_drop_pos")}>          
            {this.state.isOpen["air_drop_pos"] && <InfoWindow onCloseClick = {() =>this.onToggleOpen("air_drop_pos")}>
           <div className="map-infobox">Air Drop Position</div>
            </InfoWindow>}
            </Marker>
          
            <Marker  
            title="homePosition"
            position={{lat: this.state.mission.home_pos.latitude, lng: this.state.mission.home_pos.longitude}}
            onClick = {()=>this.onToggleOpen("home_pos")}>        
            {this.state.isOpen["home_pos"] && <InfoWindow onCloseClick = {() =>this.onToggleOpen("home_pos")}>
            <div className="map-infobox">Home Position</div>
          </InfoWindow> }          
            </Marker>
          
       {/*    let lineCoordinates = new Array();
            for(i = 0; i< this.state.mission[0].fly_zones.length; i++){ 
                lineCoordinates[i] = {{lat: this.state.mission[0].fly_zones[0].boundary_pts[i].latitude ,lng: this.state.mission[0].fly_zones[0].boundary_pts[i].longitude}} ;
            }
      
                

          
           var flightPath = new google.maps.Polyline({
          path: lineCoordinates,
          geodesic: true,
          strokeColor: '#FF0000',
          strokeOpacity: 1.0,
          strokeWeight: 2
        });
        flightPath.setMap(map);
      */}
            <Polygon
                path = {lineCoordinates} strokeOpacity= {1.0} strokeWeight= {2}
            />    
          
          {
            this.state.mission.stationary_obstacles.map((obstacle, index) => {
            return <Circle radius={obstacle.cylinder_radius} center={{lat: obstacle.latitude, lng: obstacle.longitude}}/>;
            })
          }

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
