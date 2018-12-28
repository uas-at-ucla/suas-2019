import shortid from 'shortid';
import { createMessage } from '../protobuf/loadGroundLanguage';

export default {
  addCommand: (type, options, protoInfo) => {
    return {
      type: 'ADD_COMMAND',
      payload: createCommand(type, options, protoInfo)
    }
  },
  addWaypointCommand: (options, protoInfo) => {
    return {
      type: 'ADD_COMMAND',
      payload: createCommand('WaypointCommand', options, protoInfo)
    }
  },
  deleteCommand: (index) => {
    return {
      type: 'DELETE_COMMAND',
      payload: index
    }
  },
  moveCommand: (fromIndex, toIndex) => {
    return {
      type: 'MOVE_COMMAND',
      payload: {
        fromIndex: fromIndex,
        toIndex: toIndex
      }
    }
  },
  changeCommandType: (index, oldCommand, newType, protoInfo) => {
    return {
      type: 'CHANGE_COMMAND_TYPE',
      payload: {
        index: index,
        newCommand: createCommand(newType, oldCommand, protoInfo)
      }
    }
  }
}

function createCommand(type, options, protoInfo) {
  // TODO: Use protoInfo to create new command using the relevant fields for this type of command
  let command = {};
  command[type] = createMissionObject(type, options, protoInfo);
  command.type = type;
  command.id = shortid.generate();
  createMessage('GroundCommand', command); // Verify that object correctly represents protobuf
  return command;
}

function createMissionObject(type, options, protoInfo) {
  // Custom function to recursively create object based on protobuf definition
  let missionObject;
  if (protoInfo.proto[type]) {
    // object is a protobuf defined object
    missionObject = {};
    for (let fieldName in protoInfo.proto[type].fields) {
      let field = protoInfo.proto[type].fields[fieldName];
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
