function createCommand(type, options, protoInfo) {
  // TODO: Use protoInfo to create new command using the relevant fields for this type of command
  return {
    ...options,
    type: type
  }
}

export default {
  addNothingCommand: (options, protoInfo) => {
    // a command that does nothing
    return {
      type: "ADD_COMMAND",
      payload: createCommand("Nothing", {}, protoInfo)
    }
  },
  addWaypointCommand: (options, protoInfo) => {
    return {
      type: "ADD_COMMAND",
      payload: createCommand("Waypoint", options, protoInfo)
    }
  },
  deleteCommand: (index) => {
    return {
      type: "DELETE_COMMAND",
      payload: index
    }
  },
  moveCommand: (fromIndex, toIndex) => {
    return {
      type: "MOVE_COMMAND",
      payload: {
        fromIndex: fromIndex,
        toIndex: toIndex
      }
    }
  },
  changeCommandType: (index, oldCommand, newType, protoInfo) => {
    return {
      type: "CHANGE_COMMAND_TYPE",
      payload: {
        index: index,
        newCommand: createCommand(newType, oldCommand, protoInfo)
      }
    }
  }
}
