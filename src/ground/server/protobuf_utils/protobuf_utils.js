/* USAGE:
  const loadProtobufUtils = require('./protobuf_utils/protobuf_utils');
  loadProtobufUtils((protobuf_utils) => {
    let message = "something invalid";
    let sensors = protobuf_utils.decodeSensors(message);
  });
*/

const path = require("path");
const protobuf = require("protobufjs");

class ProtobufUtils {
  // GroundProgram;
  // Sensors;
  // Goal;
  // Output;

  constructor(timelineRoot, telemetryRoot) {
    this.GroundProgram = timelineRoot.lookupType("src.controls.ground_server.timeline.GroundProgram");
    this.Sensors = telemetryRoot.lookupType("src.controls.Sensors");
    this.Goal = telemetryRoot.lookupType("src.controls.Goal");
    this.Output = telemetryRoot.lookupType("src.controls.Output");
  }

  makeGroundProgram(commands, missionAndObstacles) {
    let obstacles_proto = []
    let fieldBoundary_proto = []
    if (missionAndObstacles) {
      for (let obstacle of missionAndObstacles.obstacles.stationary_obstacles) {
        obstacles_proto.push({
          location: {
            latitude: obstacle.latitude,
            longitude: obstacle.longitude
          },
          cylinderRadius: obstacle.cylinder_radius,
          cylinderHeight: obstacle.cylinder_height
        });
      }
      for (let point of missionAndObstacles.mission.fly_zones[0].boundary_pts) {
        fieldBoundary_proto.push({
          latitude: point.latitude,
          longitude: point.longitude
        });
      }
    }
    return {
      commands: commands,
      staticObstacles: obstacles_proto,
      fieldBoundary: fieldBoundary_proto
    }
  }

  encodeGroundProgram(message) {
    return ProtobufUtils.encode(this.GroundProgram, message);
  }

  decodeSensors(message) {
    return ProtobufUtils.decode(this.Sensors, message);
  }

  decodeGoal(message) {
    return ProtobufUtils.decode(this.Goal, message);
  }

  decodeOutput(message) {
    return ProtobufUtils.decode(this.Output, message);
  }

  decodeTelemetry(telemetry) {
    let decoded = {};
    if (telemetry.sensors) {
      decoded.sensors = this.decodeSensors(telemetry.sensors);
    }
    if (telemetry.goal) {
      decoded.goal = this.decodeGoal(telemetry.goal);
    }
    if (telemetry.output) {
      decoded.output = this.decodeOutput(telemetry.output);
    }
    return decoded;
  }

  static decodeBase64(string) {
    let decoded = protobuf.util.newBuffer(protobuf.util.base64.length(string));
    protobuf.util.base64.decode(string, decoded, 0);
    return decoded;
  }

  static encodeBase64(bytes) {
    return protobuf.util.base64.encode(bytes, 0, bytes.length);
  }

  static decode(Type, message) {
    return Type.decode(ProtobufUtils.decodeBase64(message));
  }

  static encode(Type, message) {
    let errMsg = Type.verify(message);
    if (errMsg) {
      throw Error(errMsg);
    }
    return ProtobufUtils.encodeBase64(Type.encode(message).finish());
  }
}

module.exports = (callback) => {
  process.chdir("../../..")
  timelineRoot = protobuf.loadSync("src/controls/ground_server/timeline/timeline_grammar.proto");
  telemetryRoot = protobuf.loadSync("src/controls/messages.proto");
  callback(new ProtobufUtils(timelineRoot, telemetryRoot));
}

