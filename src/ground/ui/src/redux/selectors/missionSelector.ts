import { createSelector, createStructuredSelector } from "reselect";
import { createObjectSelector } from "reselect-map";
import { AppState } from "../store";

// TODO redesign needed here (too many any's rn)
// Google TypeScript with reselect

const protoInfo = createSelector(
  [state => (state as AppState).mission.timelineGrammar],
  timelineGrammar => {
    // Create objects that make it easy to get info about the proto definition
    if (!timelineGrammar) {
      return null;
    }
    let groundCommandNames = timelineGrammar.GroundCommand.oneofs.command.oneof;
    let commandAbbr: { [s: string]: string } = {};
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
          altitude: "m"
        },
        GroundProgram: {
          altitude: "ft"
        }
      },
      locationFields: ["goal", "ground_target", "photographer_location"]
    };
  }
);

const commandsById = createSelector(
  [state => (state as AppState).mission.commands],
  commands =>
    commands.reduce((map, cmd) => {
      map[cmd.id] = cmd;
      return map;
    }, {})
);

// createObjectSelector is more efficient because it only recalculates for commands that have changed
const commandMarkers = createObjectSelector(
  [commandsById, protoInfo],
  (cmd: any, protoInfo) => {
    if (!protoInfo) {
      return null;
    }
    for (let locationField of protoInfo.locationFields) {
      if (cmd[cmd.name][locationField]) {
        let label = null;
        if (cmd.type === "WaypointCommand") {
          label = {
            fontFamily: "Fontawesome",
            text: "\uf192",
            fontSize: "15px"
          };
        } else if (cmd.type === "UgvDropCommand") {
          label = {
            fontFamily: "Fontawesome",
            text: "\uf187",
            fontSize: "15px"
          };
        } else if (cmd.type === "LandAtLocationCommand") {
          label = {
            fontFamily: "Fontawesome",
            text: "\uf063",
            fontSize: "15px"
          };
        }
        let location = cmd[cmd.name][locationField];
        return {
          altitude: location.altitude,
          position: {
            lat: location.latitude,
            lng: location.longitude
          },
          label: label
        };
      }
    }
    return null; // if cmd does not have a location (e.g. a SleepCommand)
  }
);

const commandPoints = createSelector(
  [state => (state as AppState).mission.commands, commandMarkers, protoInfo],
  (commands, commandMarkers: any, protoInfo) => {
    return commands.map((cmd, index) => {
      let marker = commandMarkers[cmd.id];
      if (!marker || !protoInfo) {
        return null;
      }
      return {
        id: cmd.id,
        name: cmd.name,
        marker: marker,
        infobox: {
          position: marker.position,
          title: index + 1 + ": " + protoInfo.commandAbbr[cmd.name],
          content: "Altitude: " + marker.altitude + " ft rel"
        }
      };
    });
  }
);

const droneProgramPath = createSelector(
  [state => (state as AppState).mission.droneProgram, protoInfo],
  (droneProgram, protoInfo) => {
    if (!droneProgram || !protoInfo) {
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

const mainFlyZone = createSelector(
  [state => (state as AppState).mission.interopData],
  interopData => {
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

export default createStructuredSelector({
  protoInfo,
  commandsById,
  commandMarkers,
  commandPoints,
  droneProgramPath,
  mainFlyZone
});

function shoelace(vertices: { latitude: number; longitude: number }[]) {
  // The shoelace formula determines the area of a polygon
  let area = 0;

  for (let i = 0; i < vertices.length; i++) {
    let j = (i + 1) % vertices.length;
    area += vertices[i].longitude * vertices[j].latitude;
    area -= vertices[j].longitude * vertices[i].latitude;
  }

  return area;
}

function polygonArea(vertices: { latitude: number; longitude: number }[]) {
  return Math.abs(shoelace(vertices));
}

function polygonIsClockwise(
  vertices: { latitude: number; longitude: number }[]
) {
  // Determine whether vertices are clockwise/counterclockwise using the
  // sign of the output from the shoelace formula.
  return shoelace(vertices) < 0;
}
