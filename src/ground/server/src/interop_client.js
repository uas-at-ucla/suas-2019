// See test.js for usage example

const axios = require('axios');
const qs = require('qs');
const config = require('../config');

const FEET_PER_METER = 3.28084;
const interopSendFrequency = 2; //Hz
const sendInterval = 1000 / interopSendFrequency;

class InteropClient {
  // axiosInstance;
  // interopTelemetry;
  // telemetryCanUpload;

  constructor(axiosConfig, loginResponse, ui_io) {
    let sessionCookie = loginResponse.headers['set-cookie'][0];
    axiosConfig.headers = {'Cookie': sessionCookie};
    this.axiosInstance = axios.create(axiosConfig);
    this.interopTelemetry = null;
    this.telemetryCanUpload = true;

    setInterval(() => {
      if (this.interopTelemetry) {
        if (!this.interopTelemetry.sent) {
          this.interopTelemetry.sent = true;
          this.postTelemetry(this.interopTelemetry).then(msg => {
            if (config.verbose) console.log(msg);
            if (!this.telemetryCanUpload) {
              this.telemetryCanUpload = true;
              console.log("Now able to upload telemetry :)");
              ui_io.emit('INTEROP_UPLOAD_SUCCESS');
            }
          }).catch(error => {
            if (config.verbose) console.log(error);
            if (this.telemetryCanUpload) {
              this.telemetryCanUpload = false;
              console.log("Failing to upload telemetry!");
              ui_io.emit('INTEROP_UPLOAD_FAIL');
            }
          });
        } else {
          console.log("No new telemetry to send to interop.");
          this.interopTelemetry = null;
        }
      }
    }, sendInterval);
  }

  getMissions() {
    return this.axiosInstance.get("/missions").then(res => res.data);
  }
  
  getObstacles() {
    return this.axiosInstance.get("/obstacles").then(res => res.data);
  }
  
  postTelemetry(telemetry) {
    let config = {
      headers: {'Content-Type': 'application/x-www-form-urlencoded'}
    };
    telemetry = qs.stringify(telemetry); // convert to URL encoded string
    return this.axiosInstance.post("/telemetry", telemetry, config).then(res => res.data);
  }

  postObjectDetails(odlc) {
    return this.axiosInstance.post("/odlcs", odlc).then(res => res.data);
  }

  postObjectImage(image, odlcId) {
    return this.axiosInstance.post("/odlcs/"+odlcId+"/image", image).then(res => res.data);
  }

  newTelemetry(telemetry) {
    if (telemetry.sensors) {
      this.interopTelemetry = {
        latitude: telemetry.sensors.latitude,
        longitude: telemetry.sensors.longitude,
        altitude_msl: telemetry.sensors.altitude * FEET_PER_METER,
        uas_heading: telemetry.sensors.heading
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
    return new InteropClient(axiosConfig, response, ui_io);
  })
  .catch(error => {
    console.log("Failed to login to interop!");
    if (config.verbose) console.log(error);
    throw error;
  });
}
