// Run this file with `node test.js testName` or `npm test testName`

const tests = {
  interop: () => {
    const interopClientFactory = require('./interop_client/interop_client');
    interopClientFactory("localhost", 8000, "testadmin", "testpass")
      .then(interopClient => {
        interopClient.getMissions().then(missions => {
          console.log(JSON.stringify(missions, null, 2));
          console.log();

          // Object Detection, Classification, and Localization
          let testOdlc = {
            "type": "standard",
            "latitude": 38.1478,
            "longitude": -76.4275,
            "orientation": "n",
            "shape": "star",
            "background_color": "orange",
            "alphanumeric": "C",
            "alphanumeric_color": "black"
          }
  
          interopClient.postObjectDetails(testOdlc).then(odlc => {
            console.log("Submitted ODLC with id " + odlc.id);
            console.log();

            interopClient.postTelemetry({
              latitude: 38.149,
              longitude: -76.432,
              altitude_msl: 100,
              uas_heading: 90
            }).then(message => {
              console.log(message)
            }).catch(error => {
              console.log(error);
            });
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