import dotProp from 'dot-prop-immutable';
import { createSelector, createStructuredSelector } from 'reselect';

const initialState = {
  missionProto: null,
  commands: []
};

export default {
  reducer: (state=initialState, action) => {
    switch (action.type) {
      case 'MISSION_PROTO_LOADED': {
        return dotProp.set(state, `missionProto`, action.payload);
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
      [state => state.missionProto],
      (missionProto) => {
        // Create objects that make it easy to get info about the proto definition
        if (!missionProto) {
          return null;
        }
        let commands = {};
        for (let commandType of missionProto.Command.oneofs.command.oneof) {
          commands[commandType] = missionProto[commandType];
        }
        return {
          proto: missionProto,
          commands: commands
        };
      }
    )
  })
};
