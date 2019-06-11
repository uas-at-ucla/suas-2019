import { createSelector, createStructuredSelector } from 'reselect';
import { createObjectSelector } from 'reselect-map';

const selectors = {};

selectors.protoInfo = createSelector(
  [state => state.mission.timelineGrammar],
  (timelineGrammar) => {
    // Create objects that make it easy to get info about the proto definition
    if (!timelineGrammar) {
      return null;
    }
    let groundCommandNames = timelineGrammar.GroundCommand.oneofs.command.oneof;
    let commandAbbr = {};
    for (let commandName of groundCommandNames) {
      let commandType = timelineGrammar.GroundCommand.fields[commandName].type;
      commandAbbr[commandName] = commandType.replace("Command", "");
    }
    let droneCommandNames = timelineGrammar.DroneCommand.oneofs.command.oneof;
    for (let commandName of droneCommandNames) {
      let commandType = timelineGrammar.DroneCommand.fields[commandName].type;
      commandAbbr[commandName] = commandType.replace("Command", "");
    }
    return {
      timelineGrammar: timelineGrammar,
      commandNames: groundCommandNames,
      commandAbbr: commandAbbr,
      fieldUnits: {
        DroneProgram: {
          altitude: "m",
        },
        GroundProgram: {
          altitude: "ft",
        }
      },
      locationFields: ["goal", "ground_target", "photographer_location"]
    };
  }
);

selectors.commandsById = createSelector(
  [state => state.mission.commands],
  (commands) => commands.reduce((map, cmd) => {
    map[cmd.id] = cmd;
    return map;
  }, {})
);

// createObjectSelector is more efficient because it only recalculates for commands that have changed
selectors.commandMarkers = createObjectSelector( 
  [selectors.commandsById, selectors.protoInfo],
  (cmd, protoInfo) => {
    for (let locationField of protoInfo.locationFields) {
      if (cmd[cmd.name][locationField]) {
        let label = null;
        if (cmd.type === 'WaypointCommand') {
          label = {
            fontFamily: 'Fontawesome',
            text: '\uf192',
            fontSize: '15px'
          }
        } else if (cmd.type === 'UgvDropCommand') {
          label = {
            fontFamily: 'Fontawesome',
            text: '\uf187',
            fontSize: '15px'
          }
        } else if (cmd.type === 'LandAtLocationCommand') {
          label = {
            fontFamily: 'Fontawesome',
            text: '\uf063',
            fontSize: '15px'
          }
        }
        let location = cmd[cmd.name][locationField];
        return {
          altitude: location.altitude,
          position: {
            lat: location.latitude,
            lng: location.longitude
          },
          label: label
        }
      }
    }
    return null; // if cmd does not have a location (e.g. a SleepCommand)
  }
);

selectors.commandPoints = createSelector(
  [state => state.mission.commands, selectors.commandMarkers, selectors.protoInfo],
  (commands, commandMarkers, protoInfo) => {
    return commands.map((cmd, index) => {
      let marker = commandMarkers[cmd.id];
      if (!marker) {
        return null;
      }
      return {
        id: cmd.id,
        name: cmd.name,
        marker: marker,
        infobox: {
          position: marker.position,
          title: (index+1) + ": " + protoInfo.commandAbbr[cmd.name],
          content: "Altitude: " + marker.altitude + " ft rel"
        }
      }
    });
  }
);

selectors.droneProgramPath = createSelector(
  [state => state.mission.droneProgram, selectors.protoInfo],
  (droneProgram, protoInfo) => {
    if (!droneProgram) {
      return [];
    }
    let path = [];
    for (let cmd of droneProgram.commands) {
      for (let locationField of protoInfo.locationFields) {
        if (cmd[cmd.name][locationField]) {
          let location = cmd[cmd.name][locationField];
          path.push({
            lat: location.latitude,
            lng: location.longitude
          });
          break;
        }
      }
    }
    return path;
  }
);

selectors.mainFlyZone = createSelector(
  [state => state.mission.interopData],
  (interopData) => {
    if (!interopData) {
      return null;
    }
    let maxArea = -1;
    let mainFlyZone = null;
    for (let flyZone of interopData.mission.flyZones) {
      let area = polygonArea(flyZone.boundaryPoints);
      if (area > maxArea) {
        maxArea = area;
        mainFlyZone = flyZone;
      }
    }
    mainFlyZone.isClockwise = polygonIsClockwise(mainFlyZone.boundaryPoints);
    return mainFlyZone;
  }
);

function shoelace(vertices) {
  // The shoelace formula determines the area of a polygon
  let area = 0;

  for (let i = 0; i < vertices.length; i++) {
    let j = (i + 1) % vertices.length;
    area += vertices[i].longitude * vertices[j].latitude;
    area -= vertices[j].longitude * vertices[i].latitude;
  }

  return area;
}

function polygonArea(vertices) {
  return Math.abs(shoelace(vertices));
}

function polygonIsClockwise(vertices) {
  // Determine whether vertices are clockwise/counterclockwise using the
  // sign of the output from the shoelace formula.
  return shoelace(vertices) < 0;
}

export default createStructuredSelector(selectors);
