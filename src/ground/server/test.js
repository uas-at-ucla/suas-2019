// Run this file with `node test.js testName` or `npm test testName`

const tests = {
  interop: () => {
    const loadInteropClient = require('./src/interop_client');
    loadInteropClient("134.209.2.203:8000", "testuser", "testpass")
      .then(interopClient => {
        interopClient.getMission(2).then(missions => {
          console.log(JSON.stringify(missions, null, 2));
          console.log();

          // Object Detection, Classification, and Localization
          let testOdlc = {
            "mission": 2,
            "type": "STANDARD",
            "latitude": 38.1478,
            "longitude": -76.4275,
            "orientation": "N",
            "shape": "RECTANGLE",
            "shapeColor": "RED",
            "alphanumeric": "C",
            "alphanumericColor": "WHITE",
            "autonomous": false
          }
  
          interopClient.postObjectDetails(testOdlc).then(odlc => {
            console.log("Submitted ODLC with id " + odlc.id);
            console.log();

            interopClient.postObjectImage("./odlc.jpg", odlc.id).then(msg => {
              console.log(msg);

              interopClient.postTelemetry({
                latitude: 38.149,
                longitude: -76.432,
                altitude: 100,
                heading: 90
              }).then(message => {
                console.log(message);
                console.log("Test complete");
              }).catch(error => {
                console.log(error);
              });
            }).catch(error => {
              console.log(error);
            });
          }).catch(error => {
            console.log(error);
          });
        }).catch(error => {
          console.log(error);
        });
      }).catch(error => {
        console.log(error);
      });
  }
}

let testName = process.argv[2];
if (tests[testName]) {
  tests[testName]();
} else {
  console.log("Availble tests: " + Object.keys(tests));
}