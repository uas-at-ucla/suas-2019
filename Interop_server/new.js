const axios = require('axios');
var coockiePlus = "";
var cookie;
let res = {};
axios.get('localhost:8000/api/missions/1', {
    /* params: {
       Cookie: response.cookie
     },*/
     proxy: {
        host: '127.0.0.1',
        port: 8000
      },
     headers:{
         Cookie: "sessionid=f57uje5tdx46d1p58z1bvwhfxxelr8c5"
     } ,
     timeout: 5000
   })
.then(function (response) {
 console.log(response);
})
.catch(function (error) {
 console.log(error);
});