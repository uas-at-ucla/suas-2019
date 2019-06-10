// See test.js for usage example

const axios = require('axios');
const fs = require('fs');
const config = require('../config');

const FEET_PER_METER = 3.28084;
const interopSendFrequency = 2; //Hz
const sendInterval = 1000 / interopSendFrequency;

class InteropClient {
  // axiosInstance;
  // ui_io;
  // interopTelemetry;
  // telemetryCanUpload;
  // intervalHandle;

  constructor(axiosConfig, loginResponse, ui_io) {
    let sessionCookie = loginResponse.headers['set-cookie'][0];
    axiosConfig.headers = {'Cookie': sessionCookie};
    this.axiosInstance = axios.create(axiosConfig);
    this.ui_io = ui_io;
    this.interopTelemetry = null;
    this.telemetryCanUpload = true;
    this.intervalHandle = null;
  }

  setUploadInterval() {
    return setInterval(() => {
      if (this.interopTelemetry) {
        if (!this.interopTelemetry.sent) {
          this.postTelemetry({...this.interopTelemetry}).then(msg => {
            if (config.verbose) console.log(msg);
            if (!this.telemetryCanUpload) {
              this.telemetryCanUpload = true;
              console.log("Now able to upload telemetry :)");
              this.ui_io.emit('INTEROP_UPLOAD_SUCCESS');
            }
          }).catch(error => {
            if (config.verbose) console.log(error);
            if (this.telemetryCanUpload) {
              this.telemetryCanUpload = false;
              console.log("Failing to upload telemetry!");
              if (error.response) {
                console.log("    Response status " + error.response.status);
              } else if (error.request) {
                console.log("    No response received");
              } else {
                console.log("    Could not make request");
              }
              this.ui_io.emit('INTEROP_UPLOAD_FAIL');
            }
          });
          this.interopTelemetry.sent = true;
        } else {
          console.log("No new telemetry to send to interop.");
          clearInterval(this.intervalHandle);
          this.intervalHandle = null;
          this.interopTelemetry = null;
        }
      }
    }, sendInterval);
  }

  getMission(id) {
    return this.axiosInstance.get("/missions/"+id).then(res => res.data).catch(err => {throw err});
  }
  
  postTelemetry(telemetry) {
    return this.axiosInstance.post("/telemetry", telemetry).then(res => res.data).catch(err => {throw err});
  }

  postObjectDetails(odlc) {
    return this.axiosInstance.post("/odlcs", odlc).then(res => res.data).catch(err => {throw err});
  }

  postObjectImage(imagePath, odlcId) {
    let image = fs.readFileSync(imagePath);
    let config = {
      headers: {'Content-Type': 'image/jpeg'}
    };
    return this.axiosInstance.post("/odlcs/"+odlcId+"/image", image, config).then(res => res.data).catch(err => {throw err});
  }

  newTelemetry(telemetry) {
    if (telemetry.sensors) {
      this.interopTelemetry = {
        latitude: telemetry.sensors.latitude,
        longitude: telemetry.sensors.longitude,
        altitude: telemetry.sensors.altitude * FEET_PER_METER,
        heading: telemetry.sensors.heading
      }
      if (!this.intervalHandle) {
        console.log("Got new telemetry");
        this.intervalHandle = this.setUploadInterval();
      }
    }
  }
}

module.exports = (ip, username, password, ui_io) => {
  let axiosConfig = {
    baseURL: "http://" + ip + "/api/",
    timeout: 5000
  }

  return axios.post(
    "/login", 
    {
      "username": username,
      "password": password
    },
    axiosConfig
  )
  .then(response => {
    console.log("Logged in to interop");
    return new InteropClient(axiosConfig, response, ui_io);
  })
  .catch(error => {
    console.log("Failed to login to interop!");
    if (config.verbose) console.log(error);
    throw error;
  });
}
