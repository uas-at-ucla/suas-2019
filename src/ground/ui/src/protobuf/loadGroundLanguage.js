import protobuf from 'protobufjs';

import missionProtoFile from '../protobuf/ground_language.proto';

export default function loadGroundLanguage(dispatch) {
  protobuf.load(missionProtoFile, function(err, root) {
    let missionProto = root.toJSON().nested;
    let nested;
    while ((nested = missionProto[Object.keys(missionProto)[0]].nested)) {
      missionProto = nested;
    }
    console.log(missionProto);
    dispatch({
      type: 'MISSION_PROTO_LOADED',
      payload: missionProto
    });
  });
}