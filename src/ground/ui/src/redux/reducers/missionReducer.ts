import produce from "immer";
import { arrayMove } from "react-sortable-hoc";
import { AppAction } from "../actions/actionTypes";

interface MissionState {
  timelineGrammar?: any; // TODO type
  commands: any[]; // TODO type
  defaultAltitude: number;
  commandAnimate: { [s: string]: boolean };
  droneProgram?: any;
  missionCompiled: boolean;
  missionUploaded: boolean;
  lastDroppyCommand?: string;
  interopData?: any; // TODO type
  ugvDestination: {
    lat: number;
    lng: number;
  };
}

const initialState: MissionState = {
  timelineGrammar: undefined,
  commands: [],
  defaultAltitude: 100,
  commandAnimate: {},
  droneProgram: undefined,
  missionCompiled: false,
  missionUploaded: false,
  lastDroppyCommand: undefined,
  interopData: undefined,
  ugvDestination: { lat: 38.14617, lng: -76.42642 } // Official competition location
};

export default produce((draftState: MissionState, action: AppAction) => {
  switch (action.type) {
    case "RESET_REDUX_STATE": {
      const timelineGrammar = draftState.timelineGrammar;
      Object.assign(draftState, initialState);
      draftState.timelineGrammar = timelineGrammar;
      return;
    }
    case "TIMELINE_PROTO_LOADED": {
      draftState.timelineGrammar = action.payload;
      return;
    }
    case "INTEROP_DATA": {
      draftState.interopData = action.payload;
      return;
    }
    case "ADD_COMMAND": {
      draftState.commands.push(action.payload);
      draftState.missionCompiled = false;
      return;
    }
    case "DELETE_COMMAND": {
      draftState.commands.splice(action.payload, 1);
      draftState.missionCompiled = false;
      return;
    }
    case "REORDER_COMMAND": {
      draftState.commands = arrayMove(
        draftState.commands,
        action.payload.oldIndex,
        action.payload.newIndex
      );
      draftState.missionCompiled = false;
      return;
    }
    case "CHANGE_COMMAND_TYPE": {
      draftState.commands[action.payload.index] = action.payload.newCommand;
      draftState.missionCompiled = false;
      return;
    }
    case "CHANGE_COMMAND_FIELD": {
      const dotProp = action.payload.dotProp.split(".");
      const field = dotProp.pop() as string;
      dotProp.reduce((o: any, i: string) => o[i], draftState.commands)[field] =
        action.payload.newValue; // TODO ugly
      draftState.missionCompiled = false;
      if (action.payload.dotProp.endsWith("altitude")) {
        draftState.defaultAltitude = action.payload.newValue;
      }
      return;
    }
    case "ADD_REPEATED_FIELD": {
      const dotProp = action.payload.dotProp.split(".");
      dotProp
        .reduce((o: any, i: string) => o[i], draftState.commands)
        .push(action.payload.newObject);
      draftState.missionCompiled = false;
      return;
    }
    case "POP_REPEATED_FIELD": {
      const dotProp = action.payload.dotProp.split(".");
      dotProp.reduce((o: any, i: string) => o[i], draftState.commands).pop();
      draftState.missionCompiled = false;
      return;
    }
    case "CENTER_ON_COMMAND": {
      draftState.commandAnimate[action.payload.id] = true;
      return;
    }
    case "COMMAND_STOP_ANIMATION": {
      draftState.commandAnimate[action.payload.id] = false;
      return;
    }
    case "COMPILED_DRONE_PROGRAM":
    case "UPLOADED_DRONE_PROGRAM": {
      const droneProgram = action.payload;
      if (!droneProgram.commands) {
        droneProgram.commands = [];
      }
      droneProgram.commands.map((command: any, index: any) => {
        command.name = Object.keys(command)[0];
        command.type =
          draftState.timelineGrammar.DroneCommand.fields[command.name].type;
        command.id = index;
      });
      draftState.droneProgram = droneProgram;
      if (action.type === "UPLOADED_DRONE_PROGRAM") {
        draftState.missionUploaded = true;
      } else {
        draftState.missionCompiled = true;
        draftState.missionUploaded = false;
      }
      return;
    }
    case "DROPPY_COMMAND_RECEIVED": {
      draftState.lastDroppyCommand = action.payload;
      return;
    }
  }
}, initialState) as ((a: any, b: any) => MissionState);
