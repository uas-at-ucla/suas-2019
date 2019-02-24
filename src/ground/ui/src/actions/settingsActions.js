export default {
  updateSettings: function(settings) {
    return {
      type: "UPDATE_SETTINGS",
      payload: settings
    };
  }
};
