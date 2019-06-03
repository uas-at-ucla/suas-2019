import dotProp from 'dot-prop-immutable';
import { arrayMove } from 'react-sortable-hoc';

const initialState = {
  timelineGrammar: null,
  commands: [],
  defaultAltitude: 100,
  commandAnimate: {},
  droneProgram: null,
  missionCompiled: false,
  missionUploaded: false,
  // missionStatus: "NONE",
  lastDroppyCommand: null,
  interopData: null
};

export default function reducer(state=initialState, action) {
  switch (action.type) {
    case 'TIMELINE_PROTO_LOADED': {
      return dotProp.set(state, `timelineGrammar`, action.payload);
    }
    case 'INTEROP_DATA': {
      return dotProp.set(state, `interopData`, action.payload);
    }
    case 'ADD_COMMAND': {
      let newState = dotProp.set(state, `commands`, state.commands.concat(action.payload));
      newState.missionCompiled = false;
      return newState;
    }
    case 'DELETE_COMMAND': {
      let newState = dotProp.delete(state, `commands.${action.payload}`);
      newState.missionCompiled = false;
      return newState;
    }
    case 'REORDER_COMMAND': {
      let newState = dotProp.set(state, `commands`, arrayMove(state.commands, action.payload.oldIndex, action.payload.newIndex));
      newState.missionCompiled = false;
      return newState;
    }
    case 'CHANGE_COMMAND_TYPE': {
      let newState = dotProp.set(state, `commands.${action.payload.index}`, action.payload.newCommand);
      newState.missionCompiled = false;
      return newState;
    }
    case 'CHANGE_COMMAND_FIELD': {
      let newState = dotProp.set(state, `commands.${action.payload.dotProp}`, action.payload.newValue);
      newState.missionCompiled = false;
      if (action.payload.dotProp.endsWith('altitude')) {
        newState.defaultAltitude = action.payload.newValue;
      }
      return newState;
    }
    case 'ADD_REPEATED_FIELD': {
      let newRepeatedFields = dotProp.get(state, `commands.${action.payload.dotProp}`)
        .concat(action.payload.newObject);
        let newState = dotProp.set(state, `commands.${action.payload.dotProp}`, newRepeatedFields);
      newState.missionCompiled = false;
      return newState;
    }
    case 'POP_REPEATED_FIELD': {
      let newState = dotProp.delete(state, `commands.${action.payload.dotProp}.$end`);
      newState.missionCompiled = false;
      return newState;
    }
    case 'CENTER_ON_COMMAND': {
      return dotProp.set(state, `commandAnimate.${action.payload.id}`, true);
    }
    case 'COMMAND_STOP_ANIMATION': {
      return dotProp.set(state, `commandAnimate.${action.payload.id}`, false);
    }
    case 'COMPILED_DRONE_PROGRAM':
    case 'UPLOADED_DRONE_PROGRAM': {
      let droneProgram = action.payload;
      if (!droneProgram.commands) droneProgram.commands = [];
      droneProgram.commands.map((command, index) => {
        command.name = Object.keys(command)[0];
        command.type = state.timelineGrammar.DroneCommand.fields[command.name].type;
        command.id = index;
      });
      let newState = {...state, droneProgram: droneProgram};
      if (action.type === 'UPLOADED_DRONE_PROGRAM') {
        newState.missionUploaded = true;
      } else {
        newState.missionCompiled = true;
        newState.missionUploaded = false;
      }
      return newState;
    }
    // case 'MISSION_STATUS': {
    //   return dotProp.set(state, `missionStatus`, action.payload);
    // }
    case 'DROPPY_COMMAND_RECEIVED': {
      return dotProp.set(state, `lastDroppyCommand`, action.payload);
    }
    default: {
      return state;
    }
  }
}
