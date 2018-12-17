#! /usr/bin/env node
var cookie = "false";
var cookieobj = {};
const axios = require('axios');
var alphabet = '0';
var odlcs_1 ={  "type": "standard",
"latitude": 38.1478,
"longitude": -76.4275,
"orientation": "n",
"shape": "star",
"background_color": "orange",
"alphanumeric": "C",
"alphanumeric_color": "black"
}
var Data= "latitude=38.149&longitude=-76.432&altitude_msl=100&uas_heading=90"

function getMissions(c){
    axios.get('localhost:8000/api/missions/1', {
         params: {
           Cookie: c
         },
         proxy: {
            host: '127.0.0.1',
            port: 8000
          },
         headers:{
             Cookie: c
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
function getObstacles(c){
        axios.get('localhost:8000/api/obstacles', {
             params: {
               Cookie: c
             },
             proxy: {
                host: '127.0.0.1',
                port: 8000
              },
             headers:{
                 Cookie: c
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

    function gc (){axios ({
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
       
       
         var test = JSON.stringify(response.headers['set-cookie']);
          
         var i = test.search("sess=");
         var j = test.search(";")
         cookieobj.cookie=  test.substring(i+3, j);
      
        
         // = JSON.stringify(response.cookie);
     
        
    })
    .then(function lol () {
        
        
    })
    .catch(function (error) {
        console.log(error);
   
  
    })
    
    return;
}
function postOdlcs(c, odlcs)    
{axios ({
        method: 'post',
        url: "localhost:8000/api/odlcs",
        proxy: {
            host: '127.0.0.1',
            port: 8000
          },
        headers:{
            Cookie: c
        } ,
        timeout: 5000,
        data: odlcs
    })
    .then(function (response) {
     
     console.log(response.data);
    })
    .catch(function (error) {
     console.log(error);
    });
}
function telemetry(c, data)
{
    axios ({
        method: 'post',
        url: "localhost:8000/api/telemetry",
        proxy: {
            host: '127.0.0.1',
            port: 8000
          },
        headers:{
            Cookie: c,
            'Content-type':'application/x-www-form-urlencoded'
        } ,
        timeout: 5000,
        data:data
    })
    .then(function (response) {
    
     console.log(response.data);
    })
    .catch(function (error) {
     console.log(error);
    });
}
function getOdlcs(c)
{
    axios ({
        method: 'get',
        url: "localhost:8000/api/odlcs",
        proxy: {
            host: '127.0.0.1',
            port: 8000
          },
        headers:{
            Cookie: c,
            'Content-type':'application/json'
        } ,
        timeout: 5000,
    })
    .then(function (response) {
     
     console.log(response.data);
    })
    .catch(function (error) {
     console.log(error);
    });
}
function putOdlcs(c, alpha)
{
    axios({
        method : 'put',
        url : 'localhost:8000/api/odlcs/1',
        proxy: {
            host: '127.0.0.1',
            port: 8000
          },
        headers:{
            Cookie: c,
            'Content-type':'application/json'
        } ,
        timeout: 5000,
        data :{
            "alphanumeric": alpha
         }
    })
    .then(function (response) {
     
     console.log(response.data);
    })
    .catch(function (error) {
     console.log(error);
    });
}
gc();

module.exports = { variableName: "variableValue" };
//setTimeout(() => console.log(cookieobj.cookie), 2000);
//setTimeout(() => (getMissions(cookieobj.cookie)), 2001);
//setTimeout(() => (getObstacles(cookieobj.cookie)), 2002);
setTimeout(() => (postOdlcs(cookieobj.cookie, odlcs_1)), 2003);
setTimeout(() => (telemetry(cookieobj.cookie,Data)), 2004);
setTimeout(()=> (getOdlcs(cookieobj.cookie)),2005);
setTimeout(()=> (putOdlcs(cookieobj.cookie,alphabet)),2006);
