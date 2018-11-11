#! /usr/bin/env node

const axios = require('axios');
//var readline = require('readline-sync');

var coockiePlus = "";
var cookie;
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
   
     var test = JSON.stringify(response.headers['set-cookie']);
      
     var i = test.search("sess=");
     var j = test.search(";")
     var cookie =  test.substring(i+3, j);
     response.cookie= cookie;
     console.log (response.data + response.cookie)    
     
})

.catch(function (error) {
    console.log(error);
})
