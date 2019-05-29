const electronRequire = window.require;
const protobuf = electronRequire ? electronRequire('protobufjs') : null;
// Fix import statement in .proto file:
if (electronRequire) protobuf.Root.prototype.resolvePath = (origin, target) => {
  return target;
}

//TODO copy .proto files to app when packaging with Electron and use those files when not electron-is-dev

var root = null;
var packageNames = [];

export default function loadTimelineGrammar(dispatch) {
  if (!electronRequire) {
    return;
  }

  let prevCwd = window.process.cwd();
  window.process.chdir("../../../");
  protobuf.load("src/controls/ground_controls/timeline/timeline_grammar.proto", function(err, protoRoot) {
    window.process.chdir(prevCwd);
    if (err) {
      throw Error(err);
    }
    
    root = protoRoot;

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
    // console.log(JSON.stringify(timelineGrammar, null, 2)); // use this to update timeline_grammar.proto.json

    // Send to Redux store
    dispatch({
      type: 'TIMELINE_PROTO_LOADED',
      payload: timelineGrammar
    });
  });
}

export function createMessage(type, payload) {
  if (!electronRequire) {
    return null;
  }

  for (let packageName of packageNames) {
    let messageType;
    try {
      messageType = root.lookupType(packageName + type);
    } catch {
      continue;
    }
    var errMsg = messageType.verify(payload);
    if (errMsg)
      throw Error(errMsg);
  
    return messageType.create(payload);
  }

  throw Error("Type " + type + " not supported!");
}
