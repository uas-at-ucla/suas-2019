import dotProp from 'dot-prop-immutable';
import { createSelector } from 'reselect';

const initialState = {
  missionProto: null
}; 

export default {
  reducer: (state=initialState, action) => {
    switch (action.type) {
      case 'MISSION_PROTO_LOADED': {
        return { missionProto: action.payload };
      }
      default: {
        return state;
      }
    }
  },
  selectors: {
    getMissionProtoInfo: createSelector(
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
  }
};
