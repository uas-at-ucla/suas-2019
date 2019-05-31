import React, { Component, PureComponent } from 'react';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';
import { Container } from 'reactstrap';
import { SortableContainer } from 'react-sortable-hoc';

import missionActions from 'redux/actions/missionActions';
import { selector } from 'redux/store';
import CommandList from './CommandList';

const mapStateToProps = state => {
  let derivedData = selector(state);
  return { 
    mission: state.mission,
    protoInfo: derivedData.mission.protoInfo,
    interopData: state.mission.interopData,
    mainFlyZone: derivedData.mission.mainFlyZone
  };
};

const mapDispatchToProps = missionActions;

class MissionPlanner extends Component {
  render() {
    return (
      <div className="MissionPlanner">
        <this.Commands onSortEnd={this.reorderCommand} distance={2}/>
      </div>
    );
  }

  autoGenerate = () => {
    //console.log(this.props.interopData.mission);
    var default_alt = 150
    var default_drp_height = default_alt
    var default_height = default_alt

    var i
    for (i = 0; i < this.props.interopData.mission.waypoints.length; i++){
      var lat = this.props.interopData.mission.waypoints[i].latitude
      var long = this.props.interopData.mission.waypoints[i].longitude
      var alt = this.props.interopData.mission.waypoints[i].altitude 
      let defaultWaypointCommand = {
        goal: {
          latitude: lat,
          longitude: long,
          altitude: alt
        }
      }
      this.props.addWaypointCommand(defaultWaypointCommand, this.props.protoInfo);
    }
    var search_grid = []
    
    for (i = 0; i < this.props.interopData.mission.searchGridPoints.length; i++){
      var lat = this.props.interopData.mission.searchGridPoints[i].latitude
      var long = this.props.interopData.mission.searchGridPoints[i].longitude
      let search_point = {
        latitude: lat, 
        longitude: long
      }
      search_grid.push(search_point)
    }

    let search_command = {
      altitude: default_alt,
      survey_polygon: search_grid
    }

    this.props.addCommand("survey_command", search_command, this.props.protoInfo)
    
    var lat = this.props.interopData.mission.airDropPos.latitude
    var long = this.props.interopData.mission.airDropPos.longitude
    let airDropCommand = {
      drop_height: default_drp_height,
      ground_target: {
        latitude: lat,
        longitude: long
      } 
    }
    this.props.addCommand("ugv_drop_command", airDropCommand, this.props.protoInfo);
    
    var lat = this.props.interopData.mission.offAxisOdlcPos.latitude
    var long = this.props.interopData.mission.offAxisOdlcPos.longitude
    let off_axis_command = {
      photographer_location: {
        latitude: lat,
        longitude: long,
        altitude: default_height
      },
      subject_location: {
        latitude: lat,
        longitude: long,
      }
    }
    this.props.addCommand("off_axis_command", off_axis_command, this.props.protoInfo)

  }

  Commands = SortableContainer(() => {
    return (
      <Container fluid>
        {this.props.mission.interopData ? 
          <div>
            Mission Altitude Range:&nbsp;
            {this.props.mainFlyZone.altitudeMin} -&nbsp;
            {this.props.mainFlyZone.altitudeMax} ft
          </div> 
        : null}
        <CommandList 
          commands={this.props.mission.commands}
          className={this.props.className}
          protoInfo={this.props.protoInfo}
          commandChangers={this.commandChangers}
          mutable={true}
        />
        <Button onClick={this.addCommand} className="command-btn">Add Command</Button>
        {this.props.interopData && this.props.mission.commands.length === 0 ? <Button onClick={this.autoGenerate}>Auto-Generate</Button> : null}
      </Container>
    );
  });

  addCommand = () => {
    let defaultWaypointCommand = { goal: {
      latitude: 38.147483,
      longitude: -76.427778,
      altitude: 100
    }}
    this.props.addWaypointCommand(defaultWaypointCommand, this.props.protoInfo);
  }

  reorderCommand = ({oldIndex, newIndex}) => {
    this.props.reorderCommand(oldIndex, newIndex);
  };

  commandChangers = {
    centerMapOnCommand: (index) => {
      if (this.props.className === "SmallMissionPlanner") {
        let command = this.props.mission.commands[index];
        this.props.centerMapOnCommand(command, this.props.protoInfo);
        setTimeout(() => this.props.commandStopAnimation(command), 1000);
      }
    },

    deleteCommand: (event) => {
      let index = event.target.dataset.index;
      this.props.deleteCommand(index);
    },

    changeCommandType: (event) => {
      let index = event.target.dataset.index;
      let newType = event.target.value;
      let oldCommand = this.props.mission.commands[index];
      this.props.changeCommandType(index, oldCommand, newType, this.props.protoInfo);
    },
  
    changeNumberField: (event) => {
      let newValue = Number(event.target.value);
      if (!isNaN(newValue)) {
        let dotProp = event.target.dataset.dotProp;
        this.props.changeCommandField(dotProp, newValue);
      }
    },
  
    addRepeatedField: (event) => {
      let dotProp = event.target.dataset.dotProp;
      let type = event.target.dataset.type;
      this.props.addRepeatedField(dotProp, type, this.props.protoInfo);
    },
  
    popRepeatedField: (event) => {
      let dotProp = event.target.dataset.dotProp;
      this.props.popRepeatedField(dotProp);
    }
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(MissionPlanner);
