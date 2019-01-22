// See test.js for usage example

const axios = require('axios');
const qs = require('qs');

const getResData = response => response.data; // pattern for getting data from HTTP response

const functionExports = (axiosInstance) => {
  return {
    getMissions: () => axiosInstance.get("/missions").then(getResData),
    getObstacles: () => axiosInstance.get("/obstacles").then(getResData),
    postTelemetry: (telemetry) => {
      let config = {
        headers: {'Content-Type': 'application/x-www-form-urlencoded'}
      };
      telemetry = qs.stringify(telemetry); // convert to URL encoded string
      return axiosInstance.post("/telemetry", telemetry, config).then(getResData);
    },
    postObjectDetails: (odlc) => {
      return axiosInstance.post("/odlcs", odlc).then(getResData);
    },
    postObjectImage: (image, odlcId) => {
      return axiosInstance.post("/odlcs/"+odlcId+"/image", image).then(getResData);
    }
  };
};

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
    let sessionCookie = response.headers['set-cookie'][0];
    axiosConfig.headers = {'Cookie': sessionCookie};
    let axiosInstance = axios.create(axiosConfig);
    return functionExports(axiosInstance);
  })
  .catch(error => {
    console.log("Failed to login to interop!");
    console.log(error);
  });
}
