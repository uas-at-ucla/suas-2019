import React, { Component } from 'react';
import { Marker, InfoWindow, Circle, Polygon, Polyline, InfoBox } from 'react-google-maps';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';

import { selector } from 'redux/store';
import missionActions from 'redux/actions/missionActions';

const mapStateToProps = state => {
  let derivedData = selector(state);
  return {
    interopData: state.mission.interopData,
    protoInfo: derivedData.mission.protoInfo,
  };
};

const mapDispatchToProps = missionActions;


class InteropItems extends Component {
  render() {
    if (this.props.interopData) {
      var boxCenter = this.props.interopData.mission.fly_zones[0].boundary_pts[0];
      var boxCoordinates = [
        {lat: boxCenter.latitude+.1, lng: boxCenter.longitude+.1}, 
        {lat: boxCenter.latitude+.1, lng: boxCenter.longitude-.1},
        {lat: boxCenter.latitude-.1, lng: boxCenter.longitude-.1},
        {lat: boxCenter.latitude-.1, lng: boxCenter.longitude+.1}
      ];
      var lineCoordinates = this.props.interopData.mission.fly_zones[0].boundary_pts.map((coord) => {
        return {lat: coord.latitude, lng: coord.longitude};
      });
      lineCoordinates.reverse(); // TODO: decide if this is necessary automatically

      var searchCenterLat = 0;
      var searchCenterLng = 0;
      var searchNum = 0;
        var searchGridPoints = this.props.interopData.mission.search_grid_points.map((coord, index) => {
          searchCenterLat += coord.latitude;
          searchCenterLng += coord.longitude;
          searchNum++;
          return {lat: coord.latitude, lng: coord.longitude};
      })
      searchCenterLng = searchCenterLng/searchNum;
      searchCenterLat = searchCenterLat/searchNum;

      var airDropPos = {
        lat: this.props.interopData.mission.air_drop_pos.latitude, 
        lng: this.props.interopData.mission.air_drop_pos.longitude
      };

      var homePosition = {
        lat: this.props.interopData.mission.home_pos.latitude, 
        lng: this.props.interopData.mission.home_pos.longitude
      };

      var emergentPos = {
        lat: this.props.interopData.mission.emergent_last_known_pos.latitude, 
        lng: this.props.interopData.mission.emergent_last_known_pos.longitude
      };

      var offAxisPos = {
        lat: this.props.interopData.mission.off_axis_odlc_pos.latitude, 
        lng: this.props.interopData.mission.off_axis_odlc_pos.longitude
      };
    }

    return (
      <span>
        {this.props.interopData ? <span>
          <Marker  
            title="airDropPosition"
            position={airDropPos}
            onClick={() => this.props.onToggleOpen("air_drop_pos")}
            icon={{
              url: "https://upload.wikimedia.org/wikipedia/commons/thumb/1/17/WA_80_cm_archery_target.svg/180px-WA_80_cm_archery_target.svg.png",
              scaledSize: {width: 25, height: 25},
              anchor: window.google ? new window.google.maps.Point(12.5, 12.5) : null 
            }}
          >
            {this.props.isOpen["air_drop_pos"] && <InfoWindow onCloseClick = {() => this.props.onToggleOpen("air_drop_pos")}>
              <div className="map-infobox">
                <div>Air Drop Position</div>
                <Button size="sm" onClick={() => this.addWaypointCommand(airDropPos.lat, airDropPos.lng)}>
                  Add to Mission
                </Button>
              </div>
            </InfoWindow>}
          </Marker>
          
          <Marker  
            title="homePosition"
            position={homePosition}
            onClick={() => this.props.onToggleOpen("home_pos")}
            icon={{
              url: "http://www.clker.com/cliparts/F/t/X/o/S/p/simple-blue-house-md.png",
              scaledSize: {width: 20, height: 20},
              anchor: window.google ? new window.google.maps.Point(10, 10) : null
            }}
          >        
            {this.props.isOpen["home_pos"] && <InfoWindow onCloseClick = {() => this.props.onToggleOpen("home_pos")}>
              <div className="map-infobox">
                <div>Home Position</div>
                <Button size="sm" onClick={() => this.addWaypointCommand(homePosition.lat, homePosition.lng)}>
                  Add to Mission
                </Button>
              </div>
            </InfoWindow>}          
          </Marker>

          <Marker  
            title="emergent_last_known_pos"
            position={emergentPos}
            onClick={() => this.props.onToggleOpen("emergent_last_known_pos")}
            icon={{
              url: "https://allenhoole.co.uk/wp-content/uploads/2016/10/Person-Icon.png",
              scaledSize: {width: 20, height: 20},
              anchor: window.google ? new window.google.maps.Point(10, 10) : null
            }}
          >
            {this.props.isOpen["emergent_last_known_pos"] && <InfoWindow onCloseClick = {() => this.props.onToggleOpen("emergent_last_known_pos")}>
              <div className="map-infobox">
                <div>Emergent Object</div>
                <Button size="sm" onClick={() =>this.addWaypointCommand(emergentPos.lat, emergentPos.lng)}>
                  Add to Mission
                </Button>
              </div>
            </InfoWindow>}          
          </Marker>

          <Marker
            title="off_axis_odlc_pos"
            position={offAxisPos}
            onClick={() => this.props.onToggleOpen("off_axis_odlc_pos")}
            icon={{
              url: "https://iconsplace.com/wp-content/uploads/_icons/ffa500/256/png/slr-camera-icon-11-256.png",
              scaledSize: {width: 20, height: 20},
              anchor: window.google ? new window.google.maps.Point(10, 10) : null
            }}
          >        
            {this.props.isOpen["off_axis_odlc_pos"] && <InfoWindow onCloseClick = {() =>this.props.onToggleOpen("off_axis_odlc_pos")}>
              <div className="map-infobox">
                <div>Off Axis Position</div>
                <Button size="sm" onClick={() => this.addWaypointCommand(offAxisPos.lat, offAxisPos.lng)}>
                  Add to Mission
                </Button>
              </div>
            </InfoWindow>}          
          </Marker>

          <Polygon
            paths={[boxCoordinates, lineCoordinates]} strokeOpacity={0.8} strokeWeight={2} 
            options={{
              strokeColor: '#FF0000',
              fillColor: '#FF0000',
              strokeOpacity: 0.8,
              strokeWeight: 1,
              fillOpacity: 0.5
            }}
          /> 

          <Polygon
            path={searchGridPoints} strokeOpacity={0.5} strokeWeight={2}
            onClick={() => this.props.onToggleOpen("search")}
          />
          {this.props.isOpen["search"] && <InfoWindow defaultPosition={{lat: searchCenterLat, lng: searchCenterLng}} onCloseClick={() => this.props.onToggleOpen("search")}>
            <div className="map-infobox">
              <div>Search Area</div>
                <Button size="sm" onClick = {() => this.addWaypointCommand(searchCenterLat, searchCenterLng)}>
                  Add to Mission
                </Button>
              </div>
          </InfoWindow>}

          {this.props.interopData.obstacles.stationary_obstacles.map((obstacle, index) => 
            <Circle 
              options={{
                strokeColor: '#FF0000',
                strokeOpacity: 0.8,
                strokeWeight: 2,
                fillColor: '#FF0000',
                fillOpacity: 0.5,
              }} 
              radius={obstacle.cylinder_radius} center={{lat: obstacle.latitude, lng: obstacle.longitude}}
              onClick={() => this.props.onToggleOpen(index)}
            />
          )}

          {this.props.interopData.obstacles.stationary_obstacles.map((obstacle, index) => 
            this.props.isOpen[index] && <InfoWindow defaultPosition={{lat: obstacle.latitude, lng: obstacle.longitude}} onCloseClick={() => this.props.onToggleOpen(index)}>
              <div className="map-infobox"> 
                Obstacle {"("+obstacle.latitude + " " + obstacle.longitude + ")"}
                <br/>
                {"Height:" + obstacle.cylinder_height + " Radius: " + obstacle.cylinder_radius} 
              </div>
            </InfoWindow>
          )}

          {this.props.interopData.mission.mission_waypoints.map((coord, index) => 
            <Marker position={{lat: coord.latitude, lng: coord.longitude }} />
          )}
        </span> : null }
      </span>
    );
  }

  deleteCommand = (event) => {
    this.props.deleteCommand(event.target.dataset.index);
  }

  mapDblClick = (event) => {
    this.addWaypointCommand(event.latLng.lat(), event.latLng.lng());
  }

  addWaypointCommand = (lat, lng) => {
    let defaultWaypointCommand = {
      goal: {
        latitude: lat,
        longitude: lng,
        altitude: 100
      }
    }
    this.props.addWaypointCommand(defaultWaypointCommand, this.props.protoInfo);
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(InteropItems);
