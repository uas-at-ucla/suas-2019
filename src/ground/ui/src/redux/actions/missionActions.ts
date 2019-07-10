import shortid from "shortid";
import { createMessage } from "protobuf/timelineGrammarUtil";

export const centerMapOnCommand = (cmd: any, protoInfo: any) => {
  for (let locationField of protoInfo.locationFields) {
    let location = cmd[cmd.name][locationField];
    if (location) {
      return {
        type: "CENTER_ON_COMMAND" as const,
        payload: {
          id: cmd.id,
          pos: {
            lat: location.latitude,
            lng: location.longitude
          }
        }
      };
    }
  }
  return { type: "NONE" as const };
};

export const commandStopAnimation = (cmd: any) => ({
  type: "COMMAND_STOP_ANIMATION" as const,
  payload: { id: cmd.id }
});

export const addCommand = (name: string, options: any, protoInfo: any) => ({
  type: "ADD_COMMAND" as const,
  payload: createCommand(name, options, protoInfo)
});

export const addWaypointCommand = (options: any, protoInfo: any) =>
  addCommand("waypoint_command", options, protoInfo);
export const addFlyThroughCommand = (options: any, protoInfo: any) =>
  addCommand("fly_through_command", options, protoInfo);

export const deleteCommand = (index: number) => ({
  type: "DELETE_COMMAND" as const,
  payload: index
});

export const reorderCommand = (oldIndex: number, newIndex: number) => ({
  type: "REORDER_COMMAND" as const,
  payload: {
    oldIndex: oldIndex,
    newIndex: newIndex
  }
});

export const changeCommandType = (
  index: number,
  oldCommand: any,
  newName: string,
  protoInfo: any
) => ({
  type: "CHANGE_COMMAND_TYPE" as const,
  payload: {
    index: index,
    newCommand: createCommand(
      newName,
      setLocationFields(oldCommand[oldCommand.name], protoInfo),
      protoInfo
    )
  }
});

export const changeCommandField = (dotProp: string, newValue: any) => ({
  type: "CHANGE_COMMAND_FIELD" as const,
  payload: {
    dotProp: dotProp,
    newValue: newValue
  }
});

export const addRepeatedField = (
  dotProp: string,
  type: string,
  protoInfo: any
) => ({
  type: "ADD_REPEATED_FIELD" as const,
  payload: {
    dotProp: dotProp,
    newObject: createMissionObject(type, null, protoInfo)
  }
});

export const popRepeatedField = (dotProp: string) => ({
  type: "POP_REPEATED_FIELD" as const,
  payload: {
    dotProp: dotProp
  }
});

function createCommand(name: string, options: any, protoInfo: any) {
  let command: any = {};
  let type = protoInfo.timelineGrammar.GroundCommand.fields[name].type;
  command[name] = createMissionObject(type, options, protoInfo);
  command.type = type; // not part of protobuf, but helpful info
  command.name = name; // not part of protobuf, but helpful info
  command.id = shortid.generate(); // not part of protobuf, but helpful info
  createMessage("GroundCommand", command); // Verify that object correctly represents protobuf
  return command;
}

function createMissionObject(type: string, options: any, protoInfo: any) {
  // Custom function to recursively create object based on protobuf definition
  let missionObject: any;
  if (protoInfo.timelineGrammar[type]) {
    // object is a protobuf defined object
    missionObject = {};
    for (let fieldName in protoInfo.timelineGrammar[type].fields) {
      let field = protoInfo.timelineGrammar[type].fields[fieldName];
      if (field.rule === "repeated") {
        // repeated objects are stored in arrays
        missionObject[fieldName] = [];
        if (options && options[fieldName]) {
          for (let fieldElement of options[fieldName]) {
            missionObject[fieldName].push(
              createMissionObject(field.type, fieldElement, protoInfo)
            );
          }
        } else {
          missionObject[fieldName].push(
            createMissionObject(field.type, null, protoInfo)
          );
        }
      } else {
        missionObject[fieldName] = createMissionObject(
          field.type,
          options ? options[fieldName] : null,
          protoInfo
        );
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

function setLocationFields(options: any, protoInfo: any) {
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
  return options;
}
