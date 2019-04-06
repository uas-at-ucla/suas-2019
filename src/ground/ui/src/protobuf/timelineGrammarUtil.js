import protobuf from 'protobufjs';

import protoFile from './timeline_grammar.proto';

const packageName = 'src.controls.ground_server.timeline.';
var root = null;

export default function loadTimelineGrammar(dispatch) {
  protobuf.load(protoFile, function(err, protoRoot) {
    root = protoRoot;
    let timelineGrammar = root.toJSON().nested;
    // Unpack the nested object, which contains an inner object for each word in the package name.
    let nested;
    while ((nested = timelineGrammar[Object.keys(timelineGrammar)[0]].nested)) {
      timelineGrammar = nested;
    }
    console.log(timelineGrammar);
    // console.log(JSON.stringify(timelineGrammar, null, 2));

    // Send to Redux store
    dispatch({
      type: 'TIMELINE_PROTO_LOADED',
      payload: timelineGrammar
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
