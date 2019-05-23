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
    let commandNames = timelineGrammar.GroundCommand.oneofs.command.oneof;
    let commandAbbr = {};
    for (let commandName of commandNames) {
      let commandType = timelineGrammar.GroundCommand.fields[commandName].type;
      commandAbbr[commandName] = commandType.replace("Command", "");
    }
    return {
      timelineGrammar: timelineGrammar,
      commandNames: commandNames,
      commandAbbr: commandAbbr,
      fieldUnits: {
        altitude: "ft",
        drop_height: "ft"
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
        }
        let location = cmd[cmd.name][locationField];
        return {
          position: {
            lat: location.latitude,
            lng: location.longitude
          },
          label: label,
          options: {
            //icon: url //if u want it to look different
          }
        }
      }
    }
    return null; // if cmd does not have a location (e.g. a SleepCommand)
  }
);

selectors.commandPoints = createSelector(
  [state => state.mission.commands, selectors.commandMarkers],
  (commands, commandMarkers) => {
    return commands.map((cmd, index) => {
      let marker = commandMarkers[cmd.id];
      if (!marker) {
        return null;
      }
      return {
        id: cmd.id,
        marker: marker,
        infobox: {
          position: marker.position,
          options: {
            //enableEventPropagation: true //we might need this if there are some buttons in the infobox.
          },
          content: (index+1)
        }
      }
    });
  }
);

selectors.interopElements = createSelector(
  [state => state.mission.interopData],
  (interopData) => {
    if (interopData) {
      return {
        home_marker: {
          position: interopData.mission.home_pos
        }
        // TODO: Add map elements to represent the rest of the mission/obstacles
      }
    }
    return null;
  }
);

export default createStructuredSelector(selectors);
