export default {
  updateSettings: function(newSettings) {
    return {
      type: "UPDATE_SETTINGS",
      payload: newSettings
    };
  }
};
