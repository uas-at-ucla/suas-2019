/* USAGE:
  const loadProtobufUtils = require('./protobuf_utils/protobuf_utils');
  loadProtobufUtils((protobuf_utils) => {
    let message = "something invalid";
    let sensors = protobuf_utils.decodeSensors(message);
  });
*/

const protobuf = require("protobufjs");

function decodeBase64(string) {
  let decoded = protobuf.util.newBuffer(protobuf.util.base64.length(string));
  protobuf.util.base64.decode(string, decoded, 0);
  return decoded;
}

function encodeBase64(bytes) {
  return protobuf.util.base64.encode(bytes, 0, bytes.length);
}

function decode(Type, message) {
  return Type.decode(decodeBase64(message));
}

function encode(Type, message) {
  return encodeBase64(Type.encode(message).finish());
}

module.exports = (callback) => {
  protobuf.load("../../controls/ground_server/timeline/timeline_grammar.proto", function(err, timelineRoot) {
    if (err) throw err;
    protobuf.load("../../controls/messages.proto", function(err, telemetryRoot) {
      if (err) throw err;

      const GroundProgram = timelineRoot.lookupType("src.controls.ground_server.timeline.GroundProgram");

      const Sensors = telemetryRoot.lookupType("src.controls.Sensors");
      const Status = telemetryRoot.lookupType("src.controls.Status");
      const Goal = telemetryRoot.lookupType("src.controls.Goal");
      const Output = telemetryRoot.lookupType("src.controls.Output");

      callback({
        encodeGroundProgram: (message) => encode(GroundProgram, message),
        decodeSensors: (message) =>  decode(Sensors, message),
        decodeStatus: (message) => decode(Status, message),
        decodeGoal: (message) => decode(Goal, message),
        decodeOutput: (message) =>  decode(Output, message)
      });
    });
  });
}
