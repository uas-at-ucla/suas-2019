import React, { Component, MouseEvent } from "react";
import { Marker, InfoWindow, Polyline } from "react-google-maps";
import { Button } from "reactstrap";
import { connect } from "react-redux";

import * as missionActions from "redux/actions/missionActions";
import { selector, AppState } from "redux/store";
import google from "components/utils/GoogleMap/google";
import GoogleMap from "components/utils/GoogleMap/GoogleMap";
import InteropItems from "./InteropItems";
import VehicleMarkers from "./VehicleMarkers";

const mapStateToProps = (state: AppState) => {
  let derivedData = selector(state);
  return {
    commandAnimate: state.mission.commandAnimate,
    defaultAltitude: state.mission.defaultAltitude,
    mapCenter: state.telemetry.mapCenter,
    commandPoints: derivedData.mission.commandPoints,
    droneProgramPath: derivedData.mission.droneProgramPath,
    protoInfo: derivedData.mission.protoInfo
  };
};

const mapDispatchToProps = missionActions;

type Props = ReturnType<typeof mapStateToProps> & (typeof mapDispatchToProps);

class Map extends Component<Props> {
  private initialIsOpen: { [key: string]: boolean } = {};
  public state = {
    isOpen: this.initialIsOpen
  };

  private toggleOpen = (id: string) => {
    this.setState({
      isOpen: { ...this.state.isOpen, [id]: !this.state.isOpen[id] }
    });
  };

  private onMapClick = () => {
    if (this.state.isOpen) {
      this.setState({
        isOpen: {}
      });
    }
  };

  public render() {
    const commandPointPolyCoords = this.props.commandPoints
      .filter(p => p)
      .map(commandPoint => {
        return commandPoint ? commandPoint.marker.position : null;
      });

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
          onClick={this.onMapClick}
          onDblClick={this.mapDblClick}
        >
          <VehicleMarkers />
          <InteropItems
            isOpen={this.state.isOpen}
            toggleOpen={this.toggleOpen}
          />

          {this.props.commandPoints.map((commandPoint, index) =>
            commandPoint ? (
              <Marker
                draggable={true}
                onDragEnd={event =>
                  this.commandDragged(event, index + "." + commandPoint.name)
                }
                {...commandPoint.marker}
                key={commandPoint.id}
                onClick={() => this.toggleOpen(commandPoint.id)}
                animation={
                  this.props.commandAnimate[commandPoint.id]
                    ? google.maps.Animation.BOUNCE
                    : null
                }
              >
                {this.state.isOpen[commandPoint.id] && (
                  <InfoWindow
                    {...commandPoint.infobox}
                    onCloseClick={() => this.toggleOpen(commandPoint.id)}
                  >
                    <div className="map-infobox">
                      <div>
                        {/* TODO: add title */}
                        {commandPoint.infobox.title}
                      </div>
                      <div>{commandPoint.infobox.content}</div>

                      <Button
                        onClick={this.deleteCommand}
                        data-index={index}
                        color="danger"
                      >
                        <i
                          className="fa fa-trash"
                          style={{ pointerEvents: "none" }}
                        ></i>
                      </Button>
                    </div>
                  </InfoWindow>
                )}
              </Marker>
            ) : null
          )}
          <Polyline
            path={commandPointPolyCoords}
            options={{
              icons: [
                {
                  icon: {
                    path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
                    strokeColor: "#000000"
                  },
                  offset: "100%",
                  repeat: "200px"
                }
              ]
            }}
          />

          <Polyline
            path={this.props.droneProgramPath}
            options={{
              strokeColor: "#3355EE",
              icons: [
                {
                  icon: {
                    path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
                    strokeColor: "#3355EE"
                  },
                  offset: "100%",
                  repeat: "200px"
                }
              ]
            }}
          />
        </GoogleMap>
      </div>
    );
  }

  private deleteCommand = (event: MouseEvent<HTMLElement>) => {
    this.props.deleteCommand(Number(event.currentTarget.dataset.index));
  };

  private commandDragged = (event: any, dotProp: string) => {
    this.props.changeCommandField(
      dotProp + ".goal.latitude",
      event.latLng.lat()
    );
    this.props.changeCommandField(
      dotProp + ".goal.longitude",
      event.latLng.lng()
    );
  };

  private mapDblClick = (event: any) => {
    this.addFlyThroughCommand(event.latLng.lat(), event.latLng.lng());
  };

  private addFlyThroughCommand = (lat: number, lng: number) => {
    let defaultWaypointCommand = {
      goal: {
        latitude: lat,
        longitude: lng,
        altitude: this.props.defaultAltitude
      }
    };
    this.props.addFlyThroughCommand(
      defaultWaypointCommand,
      this.props.protoInfo
    );
  };
}

export default connect(
  mapStateToProps,
  mapDispatchToProps
)(Map);
