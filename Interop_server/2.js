const axios = require('axios');


var coockiePlus = "";
var cookie;
let res = {cookie:"a", login : "b"};
console.log(res.cookie + "before")
function getMissions(cookie){
    axios.get('localhost:8000/api/missions/1', {
        /* params: {
           Cookie: response.cookie
         },*/
         proxy: {
            host: '127.0.0.1',
            port: 8000
          },
         headers:{
             Cookie: cookie
         } ,
         timeout: 5000
       })
    .then(function (response) {
     console.log(response.data);
    })
    .catch(function (error) {
     console.log(error);
    });
}
function getObstacles(cookie){
    axios.get('localhost:8000/api/obstacles', {
         proxy: {
            host: '127.0.0.1',
            port: 8000
          },
         headers:{
             Cookie: cookie
         } ,
         timeout: 5000
       })
    .then(function (response) {
     console.log(response.data["stationary_obstacles"]);
    })
    .catch(function (error) {
     console.log(error);
    });
}
function postOdlcs(cookie){
    axios.get('localhost:8000/api/odlcs', {
         proxy: {
            host: '127.0.0.1',
            port: 8000
          },
         headers:{
             Cookie: cookie
         } ,
         timeout: 5000
       })
    .then(function (response) {
     console.log(response);
    })
    .catch(function (error) {
     console.log(error);
    });
}
//var username = readline.question("Username:?");
axios ({
    method: 'post',
    url: "localhost:8000/api/login",
    proxy: {
        host: '127.0.0.1',
        port: 8000
      },
    timeout: 5000,
    data: {
      "username": "testadmin",
      "password": "testpass"
    }   
})

.then(function getCookie_1 (response, cookie,res) {
    var res = {};
    // console.log(response);
    // console.log("login status : " + response.data + "\n this ? " +response.headers['set-cookie']);
    // coockiePlus = response.headers['set-cookie'];
    // console.log(coockiePlus)
     var test = JSON.stringify(response.headers['set-cookie']);
     //console.log("this is test: "+test);    
     var i = test.search("sess=");
     var j = test.search(";")
     var cookie =  test.substring(i+3, j);
     //console.log("\n\n\n is this it?  " + cookie)
     response.cookie= cookie;
     console.log(response.data)
     console.log(response.cookie)
     
     // console.log(coockiePlus);
  
    getMissions(response.cookie);
    //getObstacles(response.cookie);
    
    
})
.then(console.log("this "))
.catch(function (error) {
    console.log(error);
})
