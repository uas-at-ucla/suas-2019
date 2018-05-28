import React, { Component } from 'react';
import {
  SortableContainer,
  SortableElement,
  arrayMove
} from 'react-sortable-hoc';
import './MissionPlanner.css';

const METERS_PER_FOOT = 0.3048;
const oppositeOf = {'comeToStop': 'flyThrough'}

const SortableItem = SortableElement(({ command, myIndex, subMission, self }) => {
  let type = Object.keys(command)[0];
  if (type === 'subMission') type = Object.keys(command)[1];
  let fields = self.props.commandTypes[type];

  return (
    <tr
      className="command_row"
      style = {!subMission && myIndex === self.props.homeState.droneCurrentCommand ? {
        backgroundColor: "rgba(0, 0, 255, 0.35)"
      } : {}}
    >
      <td>
        <div>{myIndex + 1}</div>
      </td>
      <td><div>{type.replace('Command', '')}</div></td>
      {fields.map((field, index) => {
        return self.commandField(command, field.name, index);
      })}
      <td>
        {command.subMission && command.subMission.commands.length > 0 ?
          <div className="sub_mission">
            <div className="sub_mission_title">sub-mission:</div>
            <table>
              <tbody>
                {command.subMission.commands.map((command, index) => (
                  <SortableItem
                    key={index}
                    index={index}
                    command={command}
                    myIndex={index}
                    subMission={true}
                    self={self}
                    disabled={true}
                  />
                ))}
              </tbody>
            </table>
          </div> : null
        }
      </td>
    </tr>
  );
});

const SortableList = SortableContainer(({ commands, self }) => {
  return (
    <div className="scrollbar">
      <table id="commandListOverview">
        <tbody>
          {commands.map((command, index) => (
            <SortableItem
              key={index}
              index={index}
              command={command}
              myIndex={index}
              subMission={false}
              self={self}
              disabled={true}
            />
          ))}
        </tbody>
      </table>
    </div>
  );
});

class DroneCommands extends Component {
  render() {
    return (
      <SortableList
        commands={this.props.homeState.droneCommands}
        self={this}
      />
    );
  }

  commandField(command, field, id) {
    let type = Object.keys(command)[0];
    if (type === 'subMission') type = Object.keys(command)[1];
    let keys = field.split('.');
    let subcommand = command[type];
    let key = null;
    for (key of keys) {
      if (Array.isArray(subcommand)) {
        subcommand = subcommand[parseInt(key)];
      } else {
        type = this.props.commandTypes[type].find(el => el.name === key).type;
        subcommand = subcommand[key];
      }
      if (subcommand == null) {
        return null;
      }
    }

    if (typeof subcommand !== 'object') {
      if (type === 'bool') {
        return <td key={id}><div>{subcommand ? field : oppositeOf[field]}</div></td>;
      } else {
        return <td key={id}>
          {this.possibleUnits(key).map(units =>
            <div className={units}>
              {parseFloat(this.convertToUnitsIfDistance(units, subcommand)).toFixed(3)}
              {units !== "none" ? (units==="metric" ? "m" : "ft") : null}
            </div>
          )}
        </td>
      }
    } else {
      let fields = this.props.commandTypes[type];
      if (Array.isArray(subcommand)) {
        return (
          <td key={id}>
            <div>
            <table>
              <tbody>
                {subcommand.map((cmd, i) => 
                  <tr key={i}>
                    {fields.map((next_field, j) => {
                      return this.commandField(command, field+'.'+i+'.'+next_field.name, j);
                    })}
                  </tr>
                )}
              </tbody>
            </table>
            </div>
          </td>
        );
      }
      return (
        <td key={id}>
          <div>
          <table>
            <tbody>
              <tr>
                {fields.map((next_field, i) => {
                  return this.commandField(command, field+'.'+next_field.name, i);
                })}
              </tr>
            </tbody>
          </table>
          </div>
        </td>
      );
    }
  }

  possibleUnits(value_type) {
    if (value_type === "altitude") {
      return ["metric", "imperial"];
    }
    return ["none"];
  }

  convertToUnitsIfDistance(toUnits, value) {
    if (toUnits === "imperial") {
      return value / METERS_PER_FOOT;
    }
    return value;
  }
}

export default DroneCommands;
