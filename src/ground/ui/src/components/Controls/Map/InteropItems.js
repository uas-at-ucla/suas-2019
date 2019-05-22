import React, { Component } from 'react';
import { Marker, Circle, Polygon } from 'react-google-maps';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';

import { selector } from 'redux/store';
import missionActions from 'redux/actions/missionActions';
import MapElementWithInfo from 'components/utils/MapElementWithInfo';

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
      var boundaryCoordinates = this.props.interopData.mission.fly_zones[0].boundary_pts.map((coord) => {
        return {lat: coord.latitude, lng: coord.longitude};
      });
      if (!polygonIsClockwise(boundaryCoordinates)) {
        boundaryCoordinates.reverse();
      }

      var searchCenterLat = 0;
      var searchCenterLng = 0;
      var searchNum = this.props.interopData.mission.search_grid_points.length;
      var searchGridPoints = this.props.interopData.mission.search_grid_points.map((coord) => {
        searchCenterLat += coord.latitude;
        searchCenterLng += coord.longitude;
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
          <MapElementWithInfo
            Element={Marker} name="airDropPosition" isOpen={this.props.isOpen} toggleOpen={this.props.toggleOpen}
            position={airDropPos}
            icon={{
              url: "https://upload.wikimedia.org/wikipedia/commons/thumb/1/17/WA_80_cm_archery_target.svg/180px-WA_80_cm_archery_target.svg.png",
              scaledSize: {width: 25, height: 25},
              anchor: {x: 12.5, y: 12.5}
            }}
          >
            <div>Air Drop Position</div>
            <Button size="sm" onClick={() => this.addWaypointCommand(airDropPos.lat, airDropPos.lng)}>
              Add to Mission
            </Button>
          </MapElementWithInfo>
          
          <MapElementWithInfo  
            Element={Marker} name="homePosition" isOpen={this.props.isOpen} toggleOpen={this.props.toggleOpen}
            position={homePosition}
            icon={{
              url: "http://www.clker.com/cliparts/F/t/X/o/S/p/simple-blue-house-md.png",
              scaledSize: {width: 20, height: 20},
              anchor: {x: 10, y: 10}
            }}
          >
            <div>Home Position</div>
            <Button size="sm" onClick={() => this.addWaypointCommand(homePosition.lat, homePosition.lng)}>
              Add to Mission
            </Button>         
          </MapElementWithInfo>

          <MapElementWithInfo  
            Element={Marker} name="emergent_last_known_pos" isOpen={this.props.isOpen} toggleOpen={this.props.toggleOpen}
            position={emergentPos}
            icon={{
              url: "https://allenhoole.co.uk/wp-content/uploads/2016/10/Person-Icon.png",
              scaledSize: {width: 20, height: 20},
              anchor: {x: 10, y: 10}
            }}
          >
            <div>Emergent Object</div>
            <Button size="sm" onClick={() => this.addWaypointCommand(emergentPos.lat, emergentPos.lng)}>
              Add to Mission
            </Button>       
          </MapElementWithInfo>

          <MapElementWithInfo
            Element={Marker} name="off_axis_odlc_pos" isOpen={this.props.isOpen} toggleOpen={this.props.toggleOpen}
            position={offAxisPos}
            icon={{
              url: "https://iconsplace.com/wp-content/uploads/_icons/ffa500/256/png/slr-camera-icon-11-256.png",
              scaledSize: {width: 20, height: 20},
              anchor: {x: 10, y: 10}
            }}
          >        
            <div>Off Axis Position</div>
            <Button size="sm" onClick={() => this.addWaypointCommand(offAxisPos.lat, offAxisPos.lng)}>
              Add to Mission
            </Button>   
          </MapElementWithInfo>

          <Polygon
            paths={[boxCoordinates, boundaryCoordinates]} strokeOpacity={0.8} strokeWeight={2} 
            options={{
              strokeColor: '#FF0000',
              fillColor: '#FF0000',
              strokeOpacity: 0.8,
              strokeWeight: 1,
              fillOpacity: 0.5
            }}
          /> 

          <MapElementWithInfo
            Element={Polygon} name="search" isOpen={this.props.isOpen} toggleOpen={this.props.toggleOpen}
            path={searchGridPoints} strokeOpacity={0.5} strokeWeight={2}
            infoPosition={{lat: searchCenterLat, lng: searchCenterLng}}
          >
            <div>Search Area</div>
            <Button size="sm" onClick={() => this.addWaypointCommand(searchCenterLat, searchCenterLng)}>
              Add to Mission
            </Button>
          </MapElementWithInfo>

          {this.props.interopData.obstacles.stationary_obstacles.map((obstacle, index) => 
            <MapElementWithInfo 
              Element={Circle} name={`obstacle-${index}`} isOpen={this.props.isOpen} toggleOpen={this.props.toggleOpen}
              options={{
                strokeColor: '#FF0000',
                strokeOpacity: 0.8,
                strokeWeight: 2,
                fillColor: '#FF0000',
                fillOpacity: 0.5,
              }} 
              radius={obstacle.cylinder_radius} center={{lat: obstacle.latitude, lng: obstacle.longitude}}
              infoPosition={{lat: obstacle.latitude, lng: obstacle.longitude}}
            >
              Obstacle {"("+obstacle.latitude + " " + obstacle.longitude + ")"}
              <br/>
              {"Height:" + obstacle.cylinder_height + " Radius: " + obstacle.cylinder_radius} 
            </MapElementWithInfo>
          )}

          {this.props.interopData.mission.mission_waypoints.map((coord, index) => 
            <MapElementWithInfo
              Element={Marker} name={`waypoint-${index}`} isOpen={this.props.isOpen} toggleOpen={this.props.toggleOpen}
              position={{lat: coord.latitude, lng: coord.longitude }}
            >
              TODO
            </MapElementWithInfo>
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

function polygonIsClockwise(vertices) {
  // Determine whether vertices are clockwise/counterclockwise using the
  // sign of the output of using the shoelace formula.
  let area = 0;

  for (let i = 0; i < vertices.length; i++) {
    let j = (i + 1) % vertices.length;
    area += vertices[i].lng * vertices[j].lat;
    area -= vertices[j].lng * vertices[i].lat;
  }

  return area < 0;
}