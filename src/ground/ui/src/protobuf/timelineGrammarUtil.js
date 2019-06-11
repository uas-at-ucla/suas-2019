const electronRequire = window.require;
const protobuf = electronRequire ? electronRequire('protobufjs') : null;
const fs = electronRequire ? electronRequire('fs') : null;
const isDev = electronRequire ? electronRequire('electron-is-dev') : null;
// Fix import statement in .proto file:
if (electronRequire) protobuf.Root.prototype.resolvePath = (origin, target) => {
  return target;
}

//TODO copy .proto files to app when packaging with Electron and use those files when not electron-is-dev

var root = electronRequire ? new protobuf.Root() : null;
var packageNames = [];

function loadRootFromJson(dispatch) {
  let jsonDescriptor = require("./timeline_grammar.proto.json");
  // Send to Redux store
  dispatch({
    type: 'TIMELINE_PROTO_LOADED',
    payload: jsonDescriptor
  });
  return protobuf.Root.fromJSON({nested: jsonDescriptor});
}

export default function loadTimelineGrammar(dispatch) {
  if (!electronRequire) {
    return;
  }

  if (!isDev) { // in packaged app, which should have an updated timeline_grammar.proto.json
    root = loadRootFromJson(dispatch);
    return;
  }

  let prevCwd = window.process.cwd();
  window.process.chdir("../../../");
  
  root.load("src/controls/ground_controls/timeline/timeline_grammar.proto", {keepCase: true}, function(err) {
    window.process.chdir(prevCwd);
    if (err) {
      throw Error(err);
    }
    
    // Combine message definitions from the file and imported files
    let timelineGrammar = {}
    for (let file of root.files) {
      let packagePath = file.split('/');
      packagePath.pop();
      let protoMessages = root.toJSON().nested;
      for (let name of packagePath) {
        protoMessages = protoMessages[name].nested;
      }
      timelineGrammar = {...timelineGrammar, ...protoMessages};
      packageNames.push(packagePath.join('.')+'.');
    }
    
    // Eliminate package references to imports
    let timelineGrammarString = JSON.stringify(timelineGrammar);
    for (let packageName of packageNames) {
      timelineGrammarString = timelineGrammarString.replace(new RegExp(packageName, 'g'), '');
    }
    timelineGrammar = JSON.parse(timelineGrammarString);

    console.log(timelineGrammar);
    // update timeline_grammar.proto.json when protobuf format has changed
    fs.writeFile("src/protobuf/timeline_grammar.proto.json", JSON.stringify(timelineGrammar, null, 2), (err) => {});
    root = loadRootFromJson(dispatch); // Load the proto root from the json file. We'll do this in the packaged app where the original .proto isn't available, so we'll do it here too for consistency.
  });
}

export function createMessage(type, payload) {
  if (!electronRequire) {
    return null;
  }

  let messageType = root.lookupType(type);
  let errMsg = messageType.verify(payload);
  if (errMsg)
    throw Error("Type " + type + " not supported!");

  return messageType.create(payload);
}
