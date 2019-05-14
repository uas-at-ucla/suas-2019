import React, { Component } from 'react';
import { Marker, InfoWindow, Circle, Polyline } from 'react-google-maps';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';

import missionActions from 'redux/actions/missionActions';
import { selector } from 'redux/store';
import GoogleMap from 'components/utils/GoogleMap/GoogleMap';
import InteropItems from './InteropItems';
import DroneMarker from './DroneMarker';

const mapStateToProps = state => {
  let derivedData = selector(state);
  return {
    commandAnimate: state.mission.commandAnimate,
    mapCenter: state.telemetry.mapCenter,
    commandPoints: derivedData.mission.commandPoints,
    protoInfo: derivedData.mission.protoInfo,
    
  };
};

const mapDispatchToProps = missionActions;


class Map extends Component {
  state = {
    isOpen: {}
  };

  toggleOpen = (id) => {
    this.setState({
      isOpen: {...this.state.isOpen, [id]: !this.state.isOpen[id]}
    });
  };
 
  onMapClick = () => {
    if (this.state.isOpen) {
      this.setState({
        isOpen: {}
      })
    }
  };

  render() {
    const commandPointPolyCoords = this.props.commandPoints.filter((p) => p).map((commandPoint, index) => {
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
          center={this.props.mapCenter}
          onClick ={this.onMapClick}
          onDblClick={this.mapDblClick}
        >
          <DroneMarker/>
          <InteropItems isOpen={this.state.isOpen} toggleOpen={this.toggleOpen} />

          {this.props.commandPoints.map((commandPoint, index) => 
            commandPoint ?
              <Marker
                {...commandPoint.marker} key={commandPoint.id} onClick = {()=>this.toggleOpen(commandPoint.id)}
                animation={(this.props.commandAnimate[commandPoint.id] && window.google) ? window.google.maps.Animation.BOUNCE : null}
              >
                {this.state.isOpen[commandPoint.id] && <InfoWindow {...commandPoint.infobox} onCloseClick = {() =>this.toggleOpen(commandPoint.id)}>
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
