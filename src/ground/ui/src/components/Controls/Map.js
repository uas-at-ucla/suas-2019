import React, { Component } from 'react';
import { Marker, InfoWindow, Circle, Polygon, Polyline, InfoBox } from 'react-google-maps';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';

import GoogleMap from 'components/Utils/GoogleMap/GoogleMap';
import missionActions from 'redux/actions/missionActions';
import { selector } from 'redux/store';

const mapStateToProps = state => {
  let derivedData = selector(state);
  return {
    mission: state.mission,
    commandPoints: derivedData.mission.commandPoints,
    protoInfo: derivedData.mission.protoInfo,
    interopData: state.mission.interopData,
    telemetry: state.telemetry,
    droneMarker: derivedData.telemetry.droneMarker 
  };
};

const mapDispatchToProps = missionActions;


class Map extends Component {
  state = {
    isOpen: {}
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
    if (this.props.interopData){
    var boxCenter =  this.props.interopData.mission.fly_zones[0].boundary_pts[0];
    var boxCoordinates =[{lat: boxCenter.latitude+.1, lng: boxCenter.longitude+.1}, 
      {lat: boxCenter.latitude+.1, lng: boxCenter.longitude-.1},
      {lat: boxCenter.latitude-.1, lng: boxCenter.longitude-.1},
      {lat: boxCenter.latitude-.1, lng: boxCenter.longitude+.1}];
    var lineCoordinates =[];
    const boundaryCoordinates = this.props.interopData.mission.fly_zones[0].boundary_pts.map((coord, index) => {
      lineCoordinates[this.props.interopData.mission.fly_zones[0].boundary_pts.length - index-1] = {lat: coord.latitude, lng: coord.longitude };
      return lineCoordinates;
    })

      var searchCoordinates = [];
        var searchGridPoints = this.props.interopData.mission.search_grid_points.map((coord, index) => {
          searchCoordinates[index] = {lat: coord.latitude, lng: coord.longitude };
          return searchCoordinates[index];
      })
      
    }
    const commandPointPolyCoords = this.props.commandPoints.map((commandPoint, index) => {
      return commandPoint.marker.position;
    })

    return (
      <div className="Map">
        <GoogleMap
          defaultZoom={16}
          defaultMapTypeId="customTiles"
          defaultOptions={{
            disableDefaultUI: true,
            disableDoubleClickZoom: true,
            scaleControl: true
          }}
          center={this.props.telemetry.mapCenter}
          onClick ={this.onMapClick}
          onDblClick={this.mapDblClick}

        >
          {this.props.droneMarker ?
            <Marker {...this.props.droneMarker}>
              <Circle
                center={this.props.droneMarker.position}
                radius={this.props.droneMarker.eph}
                options={{clickable: false}}
              ></Circle>
            </Marker>
          : null}

          {this.props.interopData ? <span><Marker  
            title="airDropPosition"
            position={{lat: this.props.interopData.mission.air_drop_pos.latitude, lng: this.props.interopData.mission.air_drop_pos.longitude}}
            onClick = {()=>this.onToggleOpen("air_drop_pos")}
            icon = {{url: "https://upload.wikimedia.org/wikipedia/commons/thumb/1/17/WA_80_cm_archery_target.svg/180px-WA_80_cm_archery_target.svg.png",
            Size: {width: 40, height:40} ,
            scaledSize: {width: 25, height: 25},
            anchor: window.google ? new window.google.maps.Point(12.5, 12.5) : null }}
            >          
            {this.state.isOpen["air_drop_pos"] && <InfoWindow onCloseClick = {() =>this.onToggleOpen("air_drop_pos")}>
           <div className="map-infobox">Air Drop Position 
           <button onClick = {() =>this.addWaypointCommand(this.props.interopData.mission.air_drop_pos.latitude, this.props.interopData.mission.air_drop_pos.longitude)}>
              add to mission
            </button>
           </div>
            </InfoWindow>}
            </Marker>
          

            <Marker  
            title="homePosition"
            position={{lat: this.props.interopData.mission.home_pos.latitude, lng: this.props.interopData.mission.home_pos.longitude}}
            onClick = {()=>this.onToggleOpen("home_pos")}
            icon = {{url: "http://www.clker.com/cliparts/F/t/X/o/S/p/simple-blue-house-md.png",
            Size: {width: "40", height:40} ,
            scaledSize: {width: 20, height: 20},
            anchor: window.google ? new window.google.maps.Point(10, 10) : null }}
            >        
            {this.state.isOpen["home_pos"] && <InfoWindow onCloseClick = {() =>this.onToggleOpen("home_pos")}>
            <div className="map-infobox">Home Position
            <button onClick = {() =>this.addWaypointCommand(this.props.interopData.mission.home_pos.latitude, this.props.interopData.mission.home_pos.longitude)}>
              add
            </button>
            </div>
          </InfoWindow> }          
            </Marker>

            <Marker  
            title="emergent_last_known_pos"
            position={{lat: this.props.interopData.mission.emergent_last_known_pos.latitude, lng: this.props.interopData.mission.emergent_last_known_pos.longitude}}
            onClick = {()=>this.onToggleOpen("emergent_last_known_pos")}
            icon = {{url: "http://www.clker.com/cliparts/F/t/X/o/S/p/simple-blue-house-md.png",
            Size: {width: "40", height:40} ,
            scaledSize: {width: 20, height: 20},
            anchor: window.google ? new window.google.maps.Point(10, 10) : null }}
            >        
            {this.state.isOpen["emergent_last_known_pos"] && <InfoWindow onCloseClick = {() =>this.onToggleOpen("emergent_last_known_pos")}>
            <div className="map-infobox">Emergenct Object
            <button onClick = {() =>this.addWaypointCommand(this.props.interopData.mission.emergent_last_known_pos.latitude, this.props.interopData.mission.emergent_last_known_pos.longitude)}>
              add
            </button>
            </div>
            </InfoWindow> }          
            </Marker>

            <Marker  
            title="off_axis_odlc_pos"
            position={{lat: this.props.interopData.mission.off_axis_odlc_pos.latitude, lng: this.props.interopData.mission.off_axis_odlc_pos.longitude}}
            onClick = {()=>this.onToggleOpen("off_axis_odlc_pos")}
            icon = {{url: "http://www.clker.com/cliparts/F/t/X/o/S/p/simple-blue-house-md.png",
            Size: {width: "40", height:40} ,
            scaledSize: {width: 20, height: 20},
            anchor: window.google ? new window.google.maps.Point(10, 10) : null }}
            >        
            {this.state.isOpen["off_axis_odlc_pos"] && <InfoWindow onCloseClick = {() =>this.onToggleOpen("off_axis_odlc_pos")}>
            <div className="map-infobox">Off Axis Position
            <button onClick = {() =>this.addWaypointCommand(this.props.interopData.mission.off_axis_odlc_pos.latitude, this.props.interopData.mission.off_axis_odlc_pos.longitude)}>
              add
            </button>
            </div>
            </InfoWindow> }          
            </Marker>
          

            <Polygon
                paths = {[boxCoordinates, lineCoordinates]} strokeOpacity= {0.8} strokeWeight= {2} 
                options={{
                  strokeColor: '#FF0000',
                  fillColor: '#FF0000',
                  strokeOpacity: 0.8,
                  strokeWeight: 1,
                  fillOpacity: 0.5
              }}
            > <InfoWindow> <div className="map-infobox"> Boundaries</div> </InfoWindow></Polygon>  
           <Polygon
                paths = {[searchGridPoints, searchGridPoints]} strokeOpacity= {0.5} strokeWeight= {2}
            />  
            

          {
            this.props.interopData.obstacles.stationary_obstacles.map((obstacle, index) => {
            return <div>
            <Circle 
            options={{
            strokeColor: '#FF0000',
            strokeOpacity: 0.8,
            strokeWeight: 2,
            fillColor: '#FF0000',
            fillOpacity: 0.5,
            }} 
            radius={obstacle.cylinder_radius} center={{lat: obstacle.latitude, lng: obstacle.longitude}}
            onClick={()=> this.onToggleOpen(index)}
             >
            {this.state.isOpen[index] && <InfoWindow defaultPosition={{lat: obstacle.latitude, lng: obstacle.longitude}} onCloseClick = {() =>this.onToggleOpen(index)}>
                <div className="map-infobox"> Obstacle  </div>
              </InfoWindow>}
            </Circle>
            
             </div>
             ;
            })
          }

          {     
            this.props.interopData.obstacles.stationary_obstacles.map((obstacle, index) => {
            return this.state.isOpen[index] && <InfoWindow defaultPosition={{lat: obstacle.latitude, lng: obstacle.longitude}} onCloseClick = {() =>this.onToggleOpen(index)}>
            <div className="map-infobox"> 
            Obstacle {"("+obstacle.latitude + " " + obstacle.longitude + ")"} 
            <br/> 
            { "Height:" + obstacle.cylinder_height + " Radius: " + obstacle.cylinder_radius }  
            </div>
             </InfoWindow>
            ;
            })
          }
          {
            this.props.interopData.mission.mission_waypoints.map((coord, index) => {
              return <Marker position={{lat: coord.latitude, lng: coord.longitude }} />
          })
          }
          </span> : null }

          {this.props.commandPoints.map((commandPoint, index) => 
            commandPoint ?
              <Marker
                {...commandPoint.marker} key={commandPoint.id} onClick = {()=>this.onToggleOpen(commandPoint.id)}
                animation={(this.props.mission.commandAnimate[commandPoint.id] && window.google) ? window.google.maps.Animation.BOUNCE : null}
              >
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
                      <i className="fa fa-trash" style={{pointerEvents: "none"}}></i>
                    </Button>
                  </div>
                </InfoWindow>}
              </Marker>

            : null
          )}
                <Polyline
                 path = {commandPointPolyCoords} strokeOpacity= {1} strokeWeight= {5} 
                 options = {{
                  icons: window.google ? [{
                    icon: {
                      path: window.google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
                      strokeColor: '#000000'
                    },
                    offset: '100%',
                    repeat: '200px'
                  }] : null
                }}
                />
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
