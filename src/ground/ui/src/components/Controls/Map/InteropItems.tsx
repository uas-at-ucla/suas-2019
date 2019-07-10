import React, { Component } from "react";
import { Marker, Circle, Polygon } from "react-google-maps";
import { Button } from "reactstrap";
import { connect } from "react-redux";

import { selector, AppState } from "redux/store";
import * as missionActions from "redux/actions/missionActions";
import MapElementWithInfo from "components/utils/MapElementWithInfo";

import camera from "./icons/camera.png";
import person from "./icons/Person-Icon.png";
import bomb from "./icons/bomb_drop.png";
import wheel from "./icons/wheel.png";
import blueMarker from "./icons/blue_marker.png";

const FEET_PER_METER = 3.28084;

const mapStateToProps = (state: AppState) => {
  let derivedData = selector(state);
  return {
    interopData: state.mission.interopData,
    ugvDestination: state.mission.ugvDestination,
    protoInfo: derivedData.mission.protoInfo,
    mainFlyZone: derivedData.mission.mainFlyZone,
    defaultAltitude: state.mission.defaultAltitude,
    homeAltitude: state.telemetry.droneTelemetry
      ? state.telemetry.droneTelemetry.sensors.home_altitude
      : null
  };
};

const mapDispatchToProps = missionActions;

type Props = ReturnType<typeof mapStateToProps> &
  (typeof mapDispatchToProps) & {
    isOpen: { [key: string]: boolean };
    toggleOpen: (id: string) => void;
  };

class InteropItems extends Component<Props> {
  public render() {
    if (this.props.interopData) {
      var boxCenter = this.props.mainFlyZone.boundaryPoints[0];
      var boxCoordinates = [
        { lat: boxCenter.latitude + 0.1, lng: boxCenter.longitude + 0.1 },
        { lat: boxCenter.latitude + 0.1, lng: boxCenter.longitude - 0.1 },
        { lat: boxCenter.latitude - 0.1, lng: boxCenter.longitude - 0.1 },
        { lat: boxCenter.latitude - 0.1, lng: boxCenter.longitude + 0.1 }
      ];
      var boundaryCoordinates = this.props.mainFlyZone.boundaryPoints.map(
        (coord: any) => {
          return { lat: coord.latitude, lng: coord.longitude };
        }
      );
      if (!this.props.mainFlyZone.isClockwise) {
        boundaryCoordinates.reverse();
      }

      var flyZonePolygons = this.props.interopData.mission.flyZones.map(
        (flyZone: any) => {
          return flyZone.boundaryPoints.map((coord: any) => {
            return { lat: coord.latitude, lng: coord.longitude };
          });
        }
      );

      var searchCenterLat = 0;
      var searchCenterLng = 0;
      var searchNum = this.props.interopData.mission.searchGridPoints.length;
      var searchGridPoints = this.props.interopData.mission.searchGridPoints.map(
        (coord: any) => {
          searchCenterLat += coord.latitude;
          searchCenterLng += coord.longitude;
          return { lat: coord.latitude, lng: coord.longitude };
        }
      );
      searchCenterLng = searchCenterLng / searchNum;
      searchCenterLat = searchCenterLat / searchNum;

      var airDropPos = {
        lat: this.props.interopData.mission.airDropPos.latitude,
        lng: this.props.interopData.mission.airDropPos.longitude
      };

      var emergentPos = {
        lat: this.props.interopData.mission.emergentLastKnownPos.latitude,
        lng: this.props.interopData.mission.emergentLastKnownPos.longitude
      };

      var offAxisPos = {
        lat: this.props.interopData.mission.offAxisOdlcPos.latitude,
        lng: this.props.interopData.mission.offAxisOdlcPos.longitude
      };

      return (
        <span>
          <MapElementWithInfo
            Element={Marker}
            name="ugvDest"
            isOpen={this.props.isOpen}
            toggleOpen={this.props.toggleOpen}
            position={this.props.ugvDestination}
            defaultIcon={{
              url: wheel,
              scaledSize: { width: 25, height: 25 },
              anchor: { x: 12.5, y: 12.5 }
            }}
          >
            <div>UGV Destionation</div>
            <div>
              {this.props.ugvDestination.lat}, {this.props.ugvDestination.lng}
            </div>
          </MapElementWithInfo>

          <MapElementWithInfo
            Element={Marker}
            name="airDropPosition"
            isOpen={this.props.isOpen}
            toggleOpen={this.props.toggleOpen}
            position={airDropPos}
            defaultIcon={{
              url: bomb,
              scaledSize: { width: 25, height: 25 },
              anchor: { x: 12.5, y: 12.5 }
            }}
          >
            <div>Air Drop Position</div>
            <Button
              size="sm"
              onClick={() =>
                this.addWaypointCommand(airDropPos.lat, airDropPos.lng)
              }
            >
              Add to Mission
            </Button>
          </MapElementWithInfo>

          <MapElementWithInfo
            Element={Marker}
            name="emergent_last_known_pos"
            isOpen={this.props.isOpen}
            toggleOpen={this.props.toggleOpen}
            position={emergentPos}
            defaultIcon={{
              url: person,
              scaledSize: { width: 20, height: 20 },
              anchor: { x: 10, y: 10 }
            }}
          >
            <div>Emergent Object</div>
            <Button
              size="sm"
              onClick={() =>
                this.addWaypointCommand(emergentPos.lat, emergentPos.lng)
              }
            >
              Add to Mission
            </Button>
          </MapElementWithInfo>

          <MapElementWithInfo
            Element={Marker}
            name="off_axis_odlc_pos"
            isOpen={this.props.isOpen}
            toggleOpen={this.props.toggleOpen}
            position={offAxisPos}
            defaultIcon={{
              url: camera,
              scaledSize: { width: 20, height: 20 },
              anchor: { x: 10, y: 10 }
            }}
          >
            <div>Off Axis Position</div>
            <Button
              size="sm"
              onClick={() =>
                this.addWaypointCommand(offAxisPos.lat, offAxisPos.lng)
              }
            >
              Add to Mission
            </Button>
          </MapElementWithInfo>

          <Polygon
            paths={[boxCoordinates, boundaryCoordinates]}
            defaultOptions={{
              strokeColor: "#FF0000",
              fillColor: "#FF0000",
              strokeOpacity: 0.8,
              strokeWeight: 1,
              fillOpacity: 0.5,
              zIndex: -1
            }}
          />

          {flyZonePolygons.map((path: any, index: number) => (
            <Polygon
              key={index}
              path={path}
              defaultOptions={{
                strokeColor: "#FF0000",
                strokeOpacity: 0.8,
                strokeWeight: 1.5,
                fillOpacity: 0,
                clickable: false,
                zIndex: -1
              }}
            />
          ))}

          <Polygon
            path={searchGridPoints}
            defaultOptions={{ clickable: false, zIndex: -1 }}
          />
          {/* Use this version if SurveyCommand is supported: */}
          {/* <MapElementWithInfo
            Element={Polygon} name="search" isOpen={this.props.isOpen} toggleOpen={this.props.toggleOpen}
            path={searchGridPoints}
            infoPosition={{lat: searchCenterLat, lng: searchCenterLng}} defaultZIndex={-1}
          >
            <div>Search Area</div>
            <Button size="sm" onClick={() => this.addWaypointCommand(searchCenterLat, searchCenterLng)}>
              Add to Mission
            </Button>
          </MapElementWithInfo> */}

          {this.props.interopData.mission.stationaryObstacles.map(
            (obstacle: any, index: number) => (
              <MapElementWithInfo
                key={index}
                Element={Circle}
                name={`obstacle-${index}`}
                isOpen={this.props.isOpen}
                toggleOpen={this.props.toggleOpen}
                defaultOptions={{
                  strokeColor: "#FF0000",
                  strokeOpacity: 0.8,
                  strokeWeight: 2,
                  fillColor: "#FF0000",
                  fillOpacity: 0.5
                }}
                radius={obstacle.radius / FEET_PER_METER}
                center={{ lat: obstacle.latitude, lng: obstacle.longitude }}
                infoPosition={{
                  lat: obstacle.latitude,
                  lng: obstacle.longitude
                }}
              >
                Obstacle{" "}
                {"(" + obstacle.latitude + ", " + obstacle.longitude + ")"}
                <br />
                {"Height: " + obstacle.height + " ft AMSL"}
                <br />
                {"Radius: " + obstacle.radius + " ft"}
              </MapElementWithInfo>
            )
          )}

          {this.props.interopData.mission.waypoints.map(
            (coord: any, index: number) => (
              <MapElementWithInfo
                key={index}
                Element={Marker}
                name={`waypoint-${index}`}
                isOpen={this.props.isOpen}
                toggleOpen={this.props.toggleOpen}
                position={{ lat: coord.latitude, lng: coord.longitude }}
                icon={{
                  url: blueMarker,
                  scaledSize: { width: 25, height: 25 },
                  anchor: { x: 12.5, y: 12.5 }
                }}
              >
                <div>
                  Waypoint {index + 1}
                  <br />
                  {coord.altitude} ft AMSL{" "}
                  {this.props.homeAltitude != null
                    ? "(" +
                      (coord.altitude -
                        this.props.homeAltitude * FEET_PER_METER) +
                      ") ft rel"
                    : null}
                </div>
                <Button
                  size="sm"
                  onClick={() =>
                    this.addWaypointCommand(
                      coord.latitude,
                      coord.longitude,
                      this.props.homeAltitude != null
                        ? coord.altitude -
                            this.props.homeAltitude * FEET_PER_METER
                        : coord.altitude
                    )
                  }
                >
                  Add to Mission
                </Button>
              </MapElementWithInfo>
            )
          )}
        </span>
      );
    } else {
      return <span></span>;
    }
  }

  private addWaypointCommand = (lat: number, lng: number, alt?: number) => {
    let defaultWaypointCommand = {
      goal: {
        latitude: lat,
        longitude: lng,
        altitude: alt ? alt : this.props.defaultAltitude
      }
    };
    this.props.addWaypointCommand(defaultWaypointCommand, this.props.protoInfo);
  };
}

export default connect(
  mapStateToProps,
  mapDispatchToProps
)(InteropItems);
