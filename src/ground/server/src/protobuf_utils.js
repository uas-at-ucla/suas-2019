/* USAGE:
  const loadProtobufUtils = require('./protobuf_utils/protobuf_utils');
  loadProtobufUtils((protobuf_utils) => {
    let message = "something invalid";
    let sensors = protobuf_utils.decodeSensors(message);
  });
*/

const protobuf = require("protobufjs");
// Fix import statement in .proto file:
protobuf.Root.prototype.resolvePath = (origin, target) => {
  return target;
}

class ProtobufUtils {
  // GroundProgram;
  // Sensors;
  // Goal;
  // Output;

  constructor(timelineRoot, telemetryRoot/*, ugvRoot*/) {
    this.GroundProgram = timelineRoot.lookupType("src.controls.ground_controls.timeline.GroundProgram");
    this.DroneProgram = timelineRoot.lookupType("src.controls.ground_controls.timeline.DroneProgram");
    this.Sensors = telemetryRoot.lookupType("src.controls.Sensors");
    this.Goal = telemetryRoot.lookupType("src.controls.Goal");
    this.Output = telemetryRoot.lookupType("src.controls.Output");
    // this.UGV_Message = ugvRoot.lookupType("ugv.messages.UGV_Message");
  }

  makeGroundProgram(commands, interopData) {
    let obstacles_proto = []
    let fieldBoundary_proto = []
    if (interopData) {
      for (let obstacle of interopData.mission.stationaryObstacles) {
        obstacles_proto.push({
          location: {
            latitude: obstacle.latitude,
            longitude: obstacle.longitude
          },
          cylinder_radius: obstacle.radius,
          cylinder_height: obstacle.height
        });
      }

      let maxArea = -1;
      let mainFlyZone = null;
      for (let flyZone of interopData.mission.flyZones) {
        let area = polygonArea(flyZone.boundaryPoints);
        if (area > maxArea) {
          maxArea = area;
          mainFlyZone = flyZone;
        }
      }
      for (let point of mainFlyZone.boundaryPoints) {
        fieldBoundary_proto.push({
          latitude: point.latitude,
          longitude: point.longitude
        });
      }
    }
    return {
      commands: commands,
      static_obstacles: obstacles_proto,
      field_boundary: fieldBoundary_proto
    }
  }

  encodeGroundProgram(message) {
    return ProtobufUtils.encode(this.GroundProgram, message);
  }

  decodeDroneProgam(message) {
    return ProtobufUtils.decode(this.DroneProgram, message);
  }

  // decodeUGV_Message(message) {
  //   return ProtobufUtils.decode(this.UGV_Message, message);
  // }

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
    return Type.toObject(Type.decode(ProtobufUtils.decodeBase64(message)));
  }

  static encode(Type, message) {
    let errMsg = Type.verify(message);
    if (errMsg) {
      throw Error(errMsg);
    }
    // console.log(Type.decode(ProtobufUtils.decodeBase64(ProtobufUtils.encodeBase64(Type.encode(Type.create(message)).finish()))));
    return ProtobufUtils.encodeBase64(Type.encode(message).finish());
  }
}

module.exports = (callback) => {
  process.chdir("../../..");
  let timelineRoot = new protobuf.Root();
  timelineRoot.load("src/controls/ground_controls/timeline/timeline_grammar.proto", {keepCase: true}, (err) => {
    if (err) throw err;

    let telemetryRoot = new protobuf.Root();
    telemetryRoot.load("src/controls/messages.proto", {keepCase: true}, (err) => {
      if (err) throw err;

      // process.chdir("src/ground/server/src/ugv");
      // let ugvRoot = new protobuf.Root();
      // ugvRoot.load("messages.proto", {keepCase: true}, (err) => {
      //   if (err) throw err;
  
        callback(new ProtobufUtils(timelineRoot, telemetryRoot/*, ugvRoot*/));
      // });
    });
  });
}


function shoelace(vertices) {
  // The shoelace formula determines the area of a polygon
  let area = 0;

  for (let i = 0; i < vertices.length; i++) {
    let j = (i + 1) % vertices.length;
    area += vertices[i].longitude * vertices[j].latitude;
    area -= vertices[j].longitude * vertices[i].latitude;
  }

  return area;
}

function polygonArea(vertices) {
  return Math.abs(shoelace(vertices));
}
