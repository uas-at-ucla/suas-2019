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
        for (let commandType of timelineGrammar.GroundCommand.oneofs.command.oneof) {
          commands[commandType] = timelineGrammar[commandType];
        }
        return {
          timelineGrammar: timelineGrammar,
          commands: commands
        };
      }
    )
  })
};
