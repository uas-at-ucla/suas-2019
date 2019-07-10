import React, { Component } from "react";
import {
  Modal,
  ModalHeader,
  ModalBody,
  TabContent,
  TabPane,
  Nav,
  NavItem,
  NavLink,
  Container
} from "reactstrap";
import { connect } from "react-redux";

import { selector, AppState } from "redux/store";
import "./MissionPlannerContainer.css";
import MissionPlanner from "./MissionPlanner";
import CommandList from "./CommandList";

const mapStateToProps = (state: AppState) => {
  return {
    droneProgram: state.mission.droneProgram,
    // missionStatus: state.mission.missionStatus,
    missionUploaded: state.mission.missionUploaded,
    dropReady: state.telemetry.droneTelemetry
      ? state.telemetry.droneTelemetry.output.deploy
      : null,
    lastDroppyCommand: state.mission.lastDroppyCommand,
    ugvStatus: state.telemetry.ugvStatus,
    protoInfo: selector(state).mission.protoInfo
  };
};

type Props = ReturnType<typeof mapStateToProps>;

class MissionPlannerContainer extends Component<Props> {
  public state = {
    expand: false,
    activeTab: "plan"
  };

  private toggleTab = (tab: "plan" | "drone") => {
    if (this.state.activeTab !== tab) {
      this.setState({
        activeTab: tab
      });
    }
  };

  public render() {
    return (
      <div className="MissionPlannerContainer">
        <div className="missionPlannerHeader">
          <Nav tabs>
            <NavItem>
              <NavLink
                className={
                  this.state.activeTab === "plan" ? "active" : undefined
                }
                onClick={() => this.toggleTab("plan")}
              >
                Plan
              </NavLink>
            </NavItem>
            <NavItem>
              <NavLink
                className={
                  this.state.activeTab === "drone" ? "active" : undefined
                }
                onClick={() => this.toggleTab("drone")}
              >
                {this.props.missionUploaded ? "Uploaded" : "Compiled"} Mission
              </NavLink>
            </NavItem>
          </Nav>
          <i onClick={this.expand} className="fa fa-expand fa-lg"></i>
        </div>

        <TabContent activeTab={this.state.activeTab}>
          <div>
            Drop Status: {this.props.lastDroppyCommand} (
            {this.props.dropReady ? "Ready" : "Not ready"})
          </div>
          <div>
            UGV is Still?{" "}
            {this.props.ugvStatus
              ? this.props.ugvStatus.is_still != null
                ? this.props.ugvStatus.is_still
                  ? "YES"
                  : "NO"
                : "UNKNOWN"
              : "UNKNOWN"}
          </div>
          {/* <span>Mission Status: {this.props.missionStatus}</span> */}
          <TabPane tabId="plan">
            <div className="SmallMissionPlanner">
              <MissionPlanner
                className="SmallMissionPlanner"
                programType="GroundProgram"
              />
            </div>
          </TabPane>
          <TabPane tabId="drone">
            <div className="SmallMissionPlanner">
              <Container fluid>
                <CommandList
                  commands={
                    this.props.droneProgram
                      ? this.props.droneProgram.commands
                      : []
                  }
                  programType="DroneProgram"
                  className="SmallMissionPlanner"
                  protoInfo={this.props.protoInfo}
                  centerMapOnCommand={() => {
                    /*TODO*/
                  }}
                  mutable={false}
                />
              </Container>
            </div>
          </TabPane>
        </TabContent>

        <Modal
          isOpen={this.state.expand}
          toggle={this.close}
          className="MissionPlannerModal"
        >
          <ModalHeader toggle={this.close}>Mission Planner</ModalHeader>
          <ModalBody>
            <div className="BigMissionPlanner">
              <MissionPlanner
                className="BigMissionPlanner"
                programType="GroundProgram"
              />
            </div>
          </ModalBody>
        </Modal>
      </div>
    );
  }

  private expand = () => this.setState({ expand: true });

  private close = () => this.setState({ expand: false });
}

export default connect(mapStateToProps)(MissionPlannerContainer);
