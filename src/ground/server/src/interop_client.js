// See test.js for usage example

const axios = require('axios');
const qs = require('qs');
const config = require('../config');

const interopSendFrequency = 2; //Hz
const sendInterval = {
  [config.droneSensorsFrequency]: Math.floor(config.droneSensorsFrequency / interopSendFrequency),
  [config.droneSensorsFreqRFD900]: Math.floor(config.droneSensorsFreqRFD900 / interopSendFrequency)
}

const emptyPromise = new Promise(()=>{});

class InteropClient {
  // axiosInstance;
  // telemetryCount = 0;

  constructor(axiosConfig, loginResponse) {
    let sessionCookie = loginResponse.headers['set-cookie'][0];
    axiosConfig.headers = {'Cookie': sessionCookie};
    this.axiosInstance = axios.create(axiosConfig);
    this.telemetryCount = 0;
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

  newTelemetry(telemetry, frequency) {
    let promise = emptyPromise;
    if (telemetry.sensors) {
      if (this.telemetryCount == sendInterval[frequency]) {
        let interopTelemetry = {
          latitude: telemetry.sensors.latitude,
          longitude: telemetry.sensors.longitude,
          altitude_msl: telemetry.sensors.altitude,
          uas_heading: telemetry.sensors.heading
        }
        // console.log(interopTelemetry);
        promise = this.postTelemetry(interopTelemetry).then(msg => 
          config.verbose && console.log(msg)
        ).catch(error => {
          console.log("Failed to upload telemetry!");
          if (config.verbose) console.log(error);
          throw error;
        });
        this.telemetryCount = 0;
      }
      this.telemetryCount++;
    }
    return promise;
  }
}

module.exports = (ip, username, password) => {
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
    return new InteropClient(axiosConfig, response);
  })
  .catch(error => {
    console.log("Failed to login to interop!");
    if (config.verbose) console.log(error);
    throw error;
  });
}
