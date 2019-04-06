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
        ip: ip,
        username: username,
        password: password
      }
   };
  }
};