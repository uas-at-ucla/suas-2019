// THIS IS SLIGHTLY OUTDATED
// This is a visual aid, not to be used as actual code
// It shows how protobuf objects we are using are represented in JavaScript.

// Position2D
var Position2D = { latitude: 1.23, longitude: 1.57 };

// Position3D
var Position3D = { latitude: 1.23, longitude: 1.57, altitude: 14.3 };

// WaypointCommand
var Position3D = { latitude: 1.23, longitude: 1.57, altitude: 14.3 };
var WaypointCommand = { goal: Position3D };

// UgvDropCommand
var Position2D = { latitude: 1.23, longitude: 1.57 };
var UgvDropCommand = { groundTarget: Position2D, dropHeight: 14.3 };

// SurveyCommand
var Position2D = { latitude: 1.23, longitude: 1.57 };
var SurveyCommand = { surveyPolygon: Position2D, altitude: 14.3 };

// OffAxisCommand
var Position3D = { latitude: 1.23, longitude: 1.57, altitude: 14.3 };
var Position2D = { latitude: 1.23, longitude: 1.57 };
var OffAxisCommand = {
  photographerLocation: Position3D,
  subjectLocation: Position2D
};

// WaitCommand
var WaitCommand = { time: 60.0 };

// GotoCommand
var Position3D = { latitude: 1.23, longitude: 1.57, altitude: 14.3 };
var GotoCommand = { goal: Position3D, comeToStop: True };

// Command (using UgvDropCommand here as example)
var Position2D = { latitude: 1.23, longitude: 1.57 };
var UgvDropCommand = { groundTarget: Position2D, dropHeight: 14.3 };
var Command = { command: UgvDropCommand };

// StaticObstacle
var Position2D = { latitude: 1.23, longitude: 1.57 };
var StaticObstacle = {
  location: Position2D,
  cylinderRadius: 2.54,
  cylinderHeight: 2.54
};

// GroundProgram (using SurveyCommand here as example)
var Position2D = { latitude: 1.23, longitude: 1.57 };
var SurveyCommand = { surveyPolygon: [Position2D, Position2D], altitude: 14.3 };
var Command = { command: SurveyCommand };
var StaticObstacle = {
  location: Position2D,
  cylinderRadius: 2.54,
  cylinderHeight: 2.54
};
var GroundProgram = {
  commands: [Command, Command],
  staticObstacles: [StaticObstacle, StaticObstacle],
  fieldBoundary: [Position2D, Position2D]
};

var sampleGroundPRogram = {
  commands: [
    {
      WaypointCommand: {
        goal: {
          latitude: 1.23,
          longitude: 1.57,
          altitude: 14.3
        }
      }
    },
    {
      UgvDropCommand: {
        groundTarget: {
          latitude: 1.23,
          longitude: 1.57
        },
        dropHeight: 14.3
      }
    },
    {
      SurveyCommand: {
        surveyPolygon: [
          {
            latitude: 1.23,
            longitude: 1.57
          },
          {
            latitude: 1.23,
            longitude: 1.57
          }
        ],
        altitude: 14.3
      }
    },
    {
      OffAxisCommand: {
        photographerLocation: {
          latitude: 1.23,
          longitude: 1.57,
          altitude: 14.3
        },
        subjectLocation: {
          latitude: 1.23,
          longitude: 1.57
        }
      }
    },
    {
      WaitCommand: {
        time: 60
      }
    }
  ],
  staticObstacles: [
    {
      location: {
        latitude: 1.23,
        longitude: 1.57
      },
      cylinderRadius: 2.54,
      cylinderHeight: 2.54
    },
    {
      location: {
        latitude: 1.23,
        longitude: 1.57
      },
      cylinderRadius: 2.54,
      cylinderHeight: 2.54
    }
  ],
  fieldBoundary: [
    {
      latitude: 1.23,
      longitude: 1.57
    },
    {
      latitude: 1.23,
      longitude: 1.57
    },
    {
      latitude: 1.23,
      longitude: 1.57
    },
    {
      latitude: 1.23,
      longitude: 1.57
    }
  ]
};
