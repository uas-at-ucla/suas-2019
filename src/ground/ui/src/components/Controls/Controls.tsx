import React, { Component } from "react";
import { connect } from "react-redux";

import "./Controls.css";
import Map from "./Map/Map";
import Telemetry from "./Telemetry/Telemetry";
import DroneActions from "./DroneActions";
import MissionPlannerContainer from "./MissionPlannerContainer/MissionPlannerContainer";
import UasLogo from "components/utils/UasLogo/UasLogo";
import * as genericActions from "redux/actions/genericActions";

const mapDispatchToProps = genericActions;

type Props = typeof mapDispatchToProps;

class Controls extends Component<Props> {
  public render() {
    return (
      <div className="Controls">
        <div className="map-overlay">
          <div>
            <span className="left-side">
              <span className="top-left">
                <div onClick={this.centerOnDrone} className="logo">
                  <UasLogo />
                </div>
                <MissionPlannerContainer />
              </span>
              <span className="bottom-left">
                <DroneActions />
              </span>
            </span>
            <span className="right-side">
              <Telemetry />
            </span>
          </div>
        </div>
        <Map />
      </div>
    );
  }

  private centerOnDrone = () => {
    this.props.centerOnDrone();
  };
}

export default connect(
  null,
  mapDispatchToProps
)(Controls);
