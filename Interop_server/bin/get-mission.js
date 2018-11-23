#! /usr/bin/env node

const axios = require('axios');
const getCookie = require('./login.js')
//var read line = require('readline-sync');

var coockiePlus = "";
var cookie;

/*function getMissions(cookie){
    axios.get('localhost:8000/api/missions/1', {
         params: {
           Cookie: response.cookie
         },
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
     console.log("this is a joke, right?" + cookie + "\n\n\n");
     console.log(response.data);
    })
    .catch(function (error) {
     console.log(error);
    });
}*/

/*axios ({
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
   
     var test = JSON.stringify(response.headers['set-cookie']);
      
     var i = test.search("sess=");
     var j = test.search(";")
     var cookie =  test.substring(i+3, j);
     response.cookie= cookie;
     
     getMissions(response.cookie);
})

.catch(function (error) {
    console.log(error);
})}*/
console.log("log "+ getCookie.ish)