import protobuf from 'protobufjs';

import missionProtoFile from './timeline_grammar.proto';

const packageName = 'src.controls.ground_server.timeline.';
var root = null;

export default function loadGroundLanguage(dispatch) {
  protobuf.load(missionProtoFile, function(err, protoRoot) {
    root = protoRoot;
    let missionProto = root.toJSON().nested;
    // Unpack the nested object, which contains an inner object for each word in the package name.
    let nested;
    while ((nested = missionProto[Object.keys(missionProto)[0]].nested)) {
      missionProto = nested;
    }
    console.log(missionProto);

    // Send to Redux store
    dispatch({
      type: 'MISSION_PROTO_LOADED',
      payload: missionProto
    });
  });
}

export function createMessage(type, payload) {
  let messageType = root.lookupType(packageName + type);
  var errMsg = messageType.verify(payload);
  if (errMsg)
    throw Error(errMsg);

  return messageType.create(payload);
}