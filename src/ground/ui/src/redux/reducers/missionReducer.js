import dotProp from 'dot-prop-immutable';
import { arrayMove } from 'react-sortable-hoc';

const initialState = {
  timelineGrammar: null,
  commands: [],
  commandAnimate: {},
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
      return dotProp.set(state, `commands`, state.commands.concat(action.payload));
    }
    case 'DELETE_COMMAND': {
      return dotProp.delete(state, `commands.${action.payload}`);
    }
    case 'REORDER_COMMAND': {
      return dotProp.set(state, `commands`, arrayMove(state.commands, action.payload.oldIndex, action.payload.newIndex));
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
    case 'CENTER_ON_COMMAND': {
      return dotProp.set(state, `commandAnimate.${action.payload.id}`, true);
    }
    case 'COMMAND_STOP_ANIMATION': {
      return dotProp.set(state, `commandAnimate.${action.payload.id}`, false);
    }
    default: {
      return state;
    }
  }
}
