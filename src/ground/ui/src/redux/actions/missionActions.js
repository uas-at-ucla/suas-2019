import shortid from 'shortid';
import { createMessage } from 'protobuf/timelineGrammarUtil';

export default {
  centerMapOnCommand: (cmd, protoInfo) => {
    for (let locationField of protoInfo.locationFields) {
      let location = cmd[cmd.name][locationField];
      if (location) {
        return {
          type: 'CENTER_ON_COMMAND',
          payload: {
            id: cmd.id,
            pos: {
              lat: location.latitude,
              lng: location.longitude
            }
          }
        }
      }
    }
    return { type: "NONE" };
  },
  commandStopAnimation: (cmd) => {
    return {
      type: 'COMMAND_STOP_ANIMATION',
      payload: { id: cmd.id }
    }
  },
  addCommand: (name, options, protoInfo) => {
    return {
      type: 'ADD_COMMAND',
      payload: createCommand(name, options, protoInfo)
    }
  },
  addWaypointCommand: (options, protoInfo) => {
    return {
      type: 'ADD_COMMAND',
      payload: createCommand('waypoint_command', options, protoInfo)
    }
  },
  addFlyThroughCommand: (options, protoInfo) => {
    return {
      type: 'ADD_COMMAND',
      payload: createCommand('fly_through_command', options, protoInfo)
    }
  },
  deleteCommand: (index) => {
    return {
      type: 'DELETE_COMMAND',
      payload: index
    }
  },
  reorderCommand: (oldIndex, newIndex) => {
    return {
      type: 'REORDER_COMMAND',
      payload: {
        oldIndex: oldIndex,
        newIndex: newIndex
      }
    }
  },
  changeCommandType: (index, oldCommand, newName, protoInfo) => {
    setLocationFields(oldCommand[oldCommand.name], protoInfo);
    return {
      type: 'CHANGE_COMMAND_TYPE',
      payload: {
        index: index,
        newCommand: createCommand(newName, oldCommand[oldCommand.name], protoInfo)
      }
    }
  },
  changeCommandField: (dotProp, newValue) => {
    return {
      type: 'CHANGE_COMMAND_FIELD',
      payload: {
        dotProp: dotProp,
        newValue: newValue
      }
    }
  },
  addRepeatedField: (dotProp, type, protoInfo) => {
    return {
      type: 'ADD_REPEATED_FIELD',
      payload: {
        dotProp: dotProp,
        newObject: createMissionObject(type, null, protoInfo)
      }
    }
  },
  popRepeatedField: (dotProp) => {
    return {
      type: 'POP_REPEATED_FIELD',
      payload: {
        dotProp: dotProp,
      }
    }
  }
}

function createCommand(name, options, protoInfo) {
  let command = {};
  let type = protoInfo.timelineGrammar.GroundCommand.fields[name].type;
  command[name] = createMissionObject(type, options, protoInfo);
  command.type = type;             // not part of protobuf, but helpful info
  command.name = name;             // not part of protobuf, but helpful info
  command.id = shortid.generate(); // not part of protobuf, but helpful info
  createMessage('GroundCommand', command); // Verify that object correctly represents protobuf
  return command;
}

function createMissionObject(type, options, protoInfo) {
  // Custom function to recursively create object based on protobuf definition
  let missionObject;
  if (protoInfo.timelineGrammar[type]) {
    // object is a protobuf defined object
    missionObject = {};
    for (let fieldName in protoInfo.timelineGrammar[type].fields) {
      let field = protoInfo.timelineGrammar[type].fields[fieldName];
      if (field.rule === 'repeated') {
        // repeated objects are stored in arrays
        missionObject[fieldName] = [];
        if (options && options[fieldName]) {
          for (let fieldElement of options[fieldName]) {
            missionObject[fieldName].push(createMissionObject(field.type, fieldElement, protoInfo));
          }
        } else {
          missionObject[fieldName].push(createMissionObject(field.type, null, protoInfo));
        }
      } else {
        missionObject[fieldName] = createMissionObject(field.type, options ? options[fieldName] : null, protoInfo);
      }
    }
    createMessage(type, missionObject); // Verify that object correctly represents protobuf
  } else {
    // object is a primitive, i.e. a number or a string
    if (options != null) {
      missionObject = options;
    } else {
      // assume that the field is a number (all of them are currently)
      missionObject = 0;
    }
  }

  return missionObject;
}

function setLocationFields(options, protoInfo) {
  // Set all location fields so location is preserved when changing command types
  let location = null;
  for (let locationField of protoInfo.locationFields) {
    if (options[locationField]) {
      location = options[locationField];
      break;
    }
  }
  if (location) {
    for (let locationField of protoInfo.locationFields) {
      options[locationField] = location;
    }
  }
}
