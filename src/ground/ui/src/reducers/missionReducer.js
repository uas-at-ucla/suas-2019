import dotProp from 'dot-prop-immutable';
import { createSelector, createStructuredSelector } from 'reselect';

const initialState = {
  timelineGrammar: null,
  commands: []
};

export default {
  reducer: (state=initialState, action) => {
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
  },
  selector: createStructuredSelector({
    protoInfo: createSelector(
      [state => state.timelineGrammar],
      (timelineGrammar) => {
        // Create objects that make it easy to get info about the proto definition
        if (!timelineGrammar) {
          return null;
        }
        let commands = {};
        let commandTypes = [];
        for (let varName of timelineGrammar.GroundCommand.oneofs.command.oneof) {
          let commandType = timelineGrammar.GroundCommand.fields[varName].type;
          commands[commandType] = timelineGrammar[commandType];
          commandTypes.push(commandType);
        }
        return {
          timelineGrammar: timelineGrammar,
          commands: commands,
          commandTypes: commandTypes
        };
      }
    )
  })
};
