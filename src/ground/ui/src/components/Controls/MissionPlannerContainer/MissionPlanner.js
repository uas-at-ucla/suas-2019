import React, { Component, PureComponent } from 'react';
import { Button } from 'reactstrap';
import { connect } from 'react-redux';
import { Container, Row, Col, Input, InputGroup, InputGroupAddon } from 'reactstrap';
import { SortableContainer, SortableElement } from 'react-sortable-hoc';

import missionActions from 'redux/actions/missionActions';
import { selector } from 'redux/store';

const mapStateToProps = state => { 
  return { 
    mission: state.mission,
    protoInfo: selector(state).mission.protoInfo,
    interopData: state.mission.interopData
  };
};

const mapDispatchToProps = missionActions;

class MissionPlanner extends Component {
  render() {
    return (
      <div className="MissionPlanner">
        <this.CommandList onSortEnd={this.reorderCommand} distance={2}/>
      </div>
    );
  }

  autoGenerate = () => {
    //console.log(this.props.interopData.mission);
    //example:
    var i
    for (i = 0; i < this.props.interopData.mission.mission_waypoints.length; i++){
      var lat = this.props.interopData.mission.mission_waypoints[i].latitude
      var long = this.props.interopData.mission.mission_waypoints[i].longitude
      var alt = this.props.interopData.mission.mission_waypoints[i].altitude_msl 
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
    var default_alt = 150
    for (i = 0; i < this.props.interopData.mission.search_grid_points.length; i++){
      var lat = this.props.interopData.mission.search_grid_points[i].latitude
      var long = this.props.interopData.mission.search_grid_points[i].longitude
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
    
    var lat = this.props.interopData.mission.air_drop_pos.latitude
    var long = this.props.interopData.mission.air_drop_pos.longitude
    var default_drp_height = 150
    let airDropCommand = {
      drop_height: default_drp_height,
      ground_target: {
        latitude: lat,
        longitude: long
      } 
    }
    this.props.addCommand("ugv_drop_command", airDropCommand, this.props.protoInfo);
    
    var lat = this.props.interopData.mission.off_axis_odlc_pos.latitude
    var long = this.props.interopData.mission.off_axis_odlc_pos.longitude
    var default_height = 150
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

  CommandList = SortableContainer(() => {
    return (
      <Container fluid>
        {this.props.mission.interopData ? 
          <div>
            Mission Altitude Range:&nbsp;
            {this.props.mission.interopData.mission.fly_zones[0].altitude_msl_min} -&nbsp;
            {this.props.mission.interopData.mission.fly_zones[0].altitude_msl_max} ft
          </div> 
        : null}
        {this.props.mission.commands.map((command, index) => 
          <SortableCommand
            className={this.props.className}
            key={command.id}
            {...this.commandChangers}
            command={command}
            index={index}
            myIndex={index}
            protoInfo={this.props.protoInfo}
          ></SortableCommand>
        )}
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

const SortableCommand = SortableElement(props => <CommandRow {...props}/>);
// PureComponent improves performance b/c it only re-renders when props change
class CommandRow extends PureComponent {
  centerMapOnCommand = () => {
    this.props.centerMapOnCommand(this.props.myIndex);
  }

  render() {
    let command = this.props.command;
    let index = this.props.myIndex;
    return (
      <Row className={`MissionPlanner ${this.props.className}`} onClick={this.centerMapOnCommand}>
        <Col xs="auto" className="command-column input command-header">
          <Button className="delete-btn" color="danger" size="sm" data-index={index} onClick={this.props.deleteCommand}>
            <i className="fa fa-minus"></i>
          </Button>
        </Col>
        <Col xs="auto" className="command-column command-index command-header">{index+1}</Col>
        <Col xs="auto" className="command-column command-type command-header">
          <span className="value">
            {this.props.protoInfo.commandAbbr[command.name]}
          </span>
          <Input
            type="select" className="input" value={command.name}
            data-index={index} onChange={this.props.changeCommandType}
          >
            {this.props.protoInfo.commandNames.map(commandName =>
              <option value={commandName} key={commandName}>
                {this.props.protoInfo.commandAbbr[commandName]}
              </option>
            )}
          </Input>
        </Col>
        <Col xs="auto" className="command-column">
          <this.Field
            dotProp={index + "." + command.name}
            type={command.type}
            object={command[command.name]}
          />
        </Col>
      </Row>
    )
  }

  // Helper components
  NumberField = ({name, dotProp, value, units}) => {
    return (
      <Row>
        <Col xs="auto">
          <InputGroup className="number input">
            <InputGroupAddon addonType="prepend">{name}</InputGroupAddon>
            <Input
              style={{width: Math.min(12, value.toString().length + 3) + "ch"}}
              value={value} type="number"
              data-dot-prop={dotProp} onChange={this.props.changeNumberField}
            ></Input>
            {units ? <InputGroupAddon addonType="append">{units}</InputGroupAddon> : null}
          </InputGroup>
          <span className="value">{Math.round(value*1e4)/1e4} {units}</span>
        </Col>
      </Row>
    );
  };

  RepeatedField = ({name, dotProp, type, object}) => {
    return (
      <span>
        {object.map((element, index) =>
          <this.Field
            name={`${name} ${index+1}`} key={index}
            dotProp={dotProp + "." + index}
            type={type}
            object={element}
          />
        )}
        <Button
          className="input" data-dot-prop={dotProp}
          data-type={type} onClick={this.props.addRepeatedField}
        >+</Button>
        <Button
          className="input" data-dot-prop={dotProp}
          onClick={this.props.popRepeatedField}
        >-</Button>
      </span>
    );
  }

  Field = ({name, dotProp, type, object}) => {
    // Recursively create HTML based on protobuf definition
    let timelineGrammar = this.props.protoInfo.timelineGrammar;
    if (timelineGrammar[type]) {
      // object is a protobuf defined object
      return (
        <Row>
          {name ? <Col xs="auto" className="name">{name}:</Col> : null}
          {Object.keys(timelineGrammar[type].fields).map((fieldName) => {
            let field = timelineGrammar[type].fields[fieldName];
            let fieldDotProp = dotProp + "." + fieldName;
            let fieldProps = {
              name: fieldName,
              dotProp: fieldDotProp,
              type: field.type,
              object: object[fieldName]
            };
            return (
              <Col xs="auto" className="field-container" key={fieldName}>
                {field.rule === 'repeated' ?
                  <this.RepeatedField {...fieldProps}/>
                : field.rule === 'required' ?
                  <this.Field {...fieldProps}/>
                :
                  (() => {throw new Error("No support for timeline_grammar rule '" + field.rule + "' yet!")})()
                }
              </Col>
            );
          })}
        </Row>
      );
    } else if (type === "double") {
      return <this.NumberField name={name} dotProp={dotProp} value={object} 
        units={this.props.protoInfo.fieldUnits[name]}/>;
    } else {
      throw new Error("No support for timeline_grammar type '" + type + "' yet!");
    }
  };
}
