import dotProp from 'dot-prop-immutable';
import { createSelector, createStructuredSelector } from 'reselect';
import { createObjectSelector } from 'reselect-map';

const initialState = {
  timelineGrammar: null,
  commands: []
};

function reducer(state=initialState, action) {
  switch (action.type) {
    case 'TIMELINE_PROTO_LOADED': {
      return dotProp.set(state, `timelineGrammar`, action.payload);
    }
    case 'ADD_COMMAND': {
      return dotProp.set(state, `commands`, state.commands.concat(action.payload));
    }
    case 'DELETE_COMMAND': {
      return dotProp.delete(state, `commands.${action.payload.index}`);
    }
    case 'CHANGE_COMMAND_TYPE': {
      return dotProp.set(state, `commands.${action.payload.index}`, action.payload.newCommand);
    }
    case 'CHANGE_COMMAND_FIELD': {
      return dotProp.set(state, `commands.${action.payload.dotProp}`, action.payload.newValue);
    }
    case 'ADD_REPEATED_FIELD': {
      let newRepeatedFields = dotProp.get(state, `commands.${action.payload.dotProp}`)
        .concat(action.payload.newObject);
      return dotProp.set(state, `commands.${action.payload.dotProp}`, newRepeatedFields);
    }
    case 'POP_REPEATED_FIELD': {
      return dotProp.delete(state, `commands.${action.payload.dotProp}.$end`);
    }
    default: {
      return state;
    }
  }
}

const protoInfoSelector = createSelector(
  [state => state.timelineGrammar],
  (timelineGrammar) => {
    // Create objects that make it easy to get info about the proto definition
    if (!timelineGrammar) {
      return null;
    }
    let commands = {};
    let commandTypes = [];
    let commandAbbr = {};
    for (let varName of timelineGrammar.GroundCommand.oneofs.command.oneof) {
      let commandType = timelineGrammar.GroundCommand.fields[varName].type;
      commands[commandType] = timelineGrammar[commandType];
      commandTypes.push(commandType);
      commandAbbr[commandType] = commandType.replace("Command", "");
    }
    return {
      timelineGrammar: timelineGrammar,
      commands: commands,
      commandTypes: commandTypes,
      commandAbbr: commandAbbr,
      fieldUnits: {
        altitude: "ft",
        dropHeight: "ft"
      },
      locationFields: ["goal", "groundTarget", "photographerLocation"]
    };
  }
);

const commandsByIdSelector = createSelector(
  [state => state.commands],
  (commands) => commands.reduce((map, cmd) => {
    map[cmd.id] = cmd;
    return map;
  }, {})
);

// createObjectSelector is more efficient because it only recalculates for commands that have changed
const commandMarkersSelector = createObjectSelector( 
  [commandsByIdSelector, protoInfoSelector],
  (cmd, protoInfo) => {
    for (let locationField of protoInfo.locationFields) {
      if (cmd[cmd.type][locationField]) {
        let location = cmd[cmd.type][locationField];
        return {
          position: {
            lat: location.latitude,
            lng: location.longitude
          },
          label: cmd.type,
          options: {
            //icon: url //if u want it to look different
          }
        }
      }
    }
    return null;
  }
);

export default {
  reducer: reducer,

  selector: createStructuredSelector({
    protoInfo: protoInfoSelector,

    commandPoints: createSelector(
      [state => state.commands, commandMarkersSelector],
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
    )
  })
};
