export default {
  updateSettings: function(newSettings) {
    return {
      type: "UPDATE_SETTINGS",
      payload: newSettings
    };
  },

  connectToInterop: function(ip, username, password) {
    return {
      type: 'TRANSMIT',
      payload: {
        msg: 'CONNECT_TO_INTEROP',
        data: {
          ip: ip,
          username: username,
          password: password
        }
      }
    };
  },

  connectToGndServer: function() {
    return { type: 'CONNECT_TO_GND_SERVER' };
  },

  configureTrackyPos: function(pos) {
    return { 
      type: 'TRANSMIT', 
      payload: {
        msg: 'CONFIGURE_TRACKY_POS',
        data: pos
      }
    };
  },

  logReduxState: function() {
    return { type: 'LOG_REDUX_STATE' };
  }
};