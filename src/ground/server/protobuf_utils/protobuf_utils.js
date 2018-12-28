/* USAGE:
  require(./protobuf_utils/protobuf_utils)((protobuf_utils) => {
    let message = "something invalid";
    let sensors = protobuf_utils.decodeSensors(message);
  });
*/

const protobuf = require("protobufjs");

// Rename built-in functions
const decodeBase64 = atob;
const encodeBase64 = btoa;

function decode(Type, message) {
  return Type.decode(decodeBase64(message));
}

function encode(Type, message) {
  return encodeBase64(Type.encode(message).finish());
}

module.exports = (callback) => {
  protobuf.load("../../../controls/ground_server/timeline/timeline_grammar.proto", function(err, timelineRoot) {
    protobuf.load("../../../controls/messages.proto", function(err, telemetryRoot) {
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
