// See test.js for usage example

const axios = require('axios');
const qs = require('qs');
const constants = require('../utils/constants');

const interopSendFrequency = 2; //Hz
const sendInterval = Math.floor(constants.droneTelemetryFrequency / interopSendFrequency);

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

  newTelemetry(droneTelemetry) {
    if (droneTelemetry.sensors) {
      if (this.telemetryCount == sendInterval) {
        let interopTelemetry = {
          latitude: droneTelemetry.sensors.latitude,
          longitude: droneTelemetry.sensors.longitude,
          altitude_msl: droneTelemetry.sensors.altitude,
          uas_heading: droneTelemetry.sensors.heading
        }
        // console.log(interopTelemetry);
        this.postTelemetry(interopTelemetry).then(
          msg => console.log(msg)
        );
        this.telemetryCount = 0;
      }
      this.telemetryCount++;
    }
  }
}

module.exports = (ip, port, username, password) => {
  let axiosConfig = {
    baseURL: "http://" + ip + ":" + port + "/api/",
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
    console.log(error);
  });
}
