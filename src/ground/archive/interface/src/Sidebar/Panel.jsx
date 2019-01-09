import React, { Component } from "react";
import MissionPlanner from "./MissionPlanner";
import MissionPlannerOverview from "./MissionPlannerOverview";
import DroneCommands from "./DroneCommands";

class Panel extends Component {

  state = {commandView: 'plan'};

  render() {
    let header = (
      <h4>
        <span>
          <button
            onClick={() => this.setState({commandView: 'plan'})}
            className={`btn ${this.state.commandView !== 'plan' ? "btn-outline-secondary" : "btn-secondary"}`}
          >
            Plan
          </button>
          <button
            onClick={() => this.setState({commandView: 'drone'})}
            className={`btn ${this.state.commandView !== 'drone' ? "btn-outline-secondary" : "btn-secondary"}`}
          >
            Drone
          </button>
        </span>
        <button
          className="panel-expand"
          data-toggle="modal"
          data-target={`#${this.props.id}-modal`}
        >
          <i className="fa fa-expand" aria-hidden="true" />
        </button>
      </h4>
    );

    return (
      <div className="Panel" id={this.props.id}>
        <div className="card text-white">
          <div className="card-body">
            {header}
            {this.state.commandView === 'plan' ?
              <MissionPlannerOverview
                homeState={this.props.homeState}
                setHomeState={this.props.setHomeState}
                commandTypes={this.props.commandTypes}
              /> :
              <DroneCommands
                homeState={this.props.homeState}
                setHomeState={this.props.setHomeState}
                commandTypes={this.props.commandTypes}
              />
            }
          </div>
        </div>

        <div className="modal fade" id={`${this.props.id}-modal`}>
          <div className="modal-dialog modal-lg">
            <div className="modal-content">
              <div className="modal-header">
                <h5 className="modal-title">{this.props.title}</h5>
                <button className="close" data-dismiss="modal">
                  &times;
                </button>
              </div>
              <div className="modal-body">
                <MissionPlanner
                  homeState={this.props.homeState}
                  setHomeState={this.props.setHomeState}
                  makeCommand={this.props.makeCommand}
                  commandTypes={this.props.commandTypes}
                  getCommandPosKey={this.props.getCommandPosKey}
                  socketEmit={this.props.socketEmit}
                />
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }
}

export default Panel;
