import React, { Component } from 'react';
import { Modal, ModalHeader, ModalBody, TabContent, TabPane, Nav, NavItem, NavLink } from 'reactstrap';
import { connect } from 'react-redux';

import { selector } from 'redux/store';
import './MissionPlannerContainer.css';
import MissionPlanner from './MissionPlanner';
import CommandList from './CommandList';

const mapStateToProps = state => {
  return { 
    droneProgram: state.mission.droneProgram,
    // missionStatus: state.mission.missionStatus,
    missionUploaded: state.mission.missionUploaded,
    dropReady: state.telemetry.droneTelemetry ? state.telemetry.droneTelemetry.output.deploy : null,
    lastDroppyCommand: state.mission.lastDroppyCommand,
    ugvStatus: state.telemetry.ugvStatus,
    protoInfo: selector(state).mission.protoInfo
  };
};

class MissionPlannerContainer extends Component {
  state = {
    expand: false,
    activeTab: 'plan'
  }

  toggleTab = (tab) => {
    if (this.state.activeTab !== tab) {
      this.setState({
        activeTab: tab
      });
    }
  }

  render() {
    return (
      <div className="MissionPlannerContainer">
        <div className="missionPlannerHeader">
          <Nav tabs>
            <NavItem>
              <NavLink
                className={this.state.activeTab === 'plan' ? "active" : null}
                onClick={() => this.toggleTab('plan')}
              >
                Plan
              </NavLink>
            </NavItem>
            <NavItem>
              <NavLink
                className={this.state.activeTab === 'drone' ? "active" : null}
                onClick={() => this.toggleTab('drone')}
              >
                {this.props.missionUploaded ? "Uploaded" : "Compiled"} Mission
              </NavLink>
            </NavItem>
          </Nav>
          <i onClick={this.expand} className="fa fa-expand fa-lg"></i>
        </div>

        <TabContent activeTab={this.state.activeTab}>
          <div>Drop Status: {this.props.lastDroppyCommand} ({this.props.dropReady ? "Ready" : "Not ready"})</div>
          <div>UGV is Still? {this.props.ugvStatus ? this.props.ugvStatus.is_still != null ? (this.props.ugvStatus.is_still ? "YES" : "NO") : "UNKNOWN" : "UNKNOWN"}</div>
          {/* <span>Mission Status: {this.props.missionStatus}</span> */}
          <TabPane tabId="plan">
            <div className="SmallMissionPlanner">
              <MissionPlanner className="SmallMissionPlanner"/>
            </div>
          </TabPane>
          <TabPane tabId="drone">
            <div className="SmallMissionPlanner">
              <CommandList 
                commands={this.props.droneProgram ? this.props.droneProgram.commands : []}
                className="SmallMissionPlanner"
                protoInfo={this.props.protoInfo}
                commandChangers={{centerMapOnCommand: ()=>{/*TODO*/}}}
              />
            </div>
          </TabPane>
        </TabContent>
        
        <Modal isOpen={this.state.expand} toggle={this.close} className="MissionPlannerModal">
          <ModalHeader toggle={this.close}>Mission Planner</ModalHeader>
          <ModalBody>
            <div className="BigMissionPlanner">
              <MissionPlanner className="BigMissionPlanner"/>
            </div>
          </ModalBody>
        </Modal>

      </div>
    );
  }

  expand = () => this.setState({expand: true});

  close = () => this.setState({expand: false});
}

export default connect(mapStateToProps)(MissionPlannerContainer);