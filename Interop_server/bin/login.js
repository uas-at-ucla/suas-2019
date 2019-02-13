#! /usr/bin/env node
var cookie = "false";
var readlineSync =require ('readline-sync');
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
function switchTest(){
   console.log(cookieobj.cookie);
   /* var f;
    var index;
    var functions;
functions = ['getMission', 'getObstacles', 'postOdlcs','getOdlcs', 'putOdlcs','telemetry','exit']
index = readlineSync.keyInSelect(functions, 'Which function do you wanna test? ');
f= functions[index];

console.log('Ok, ' + functions[index] +' is being tested.');

switch(f)
{
    case 'getMission':
    getMissions(cookieobj.cookie);
    
    break;
    case 'getObstacles':
    getObstacles(cookieobj.cookie);
    
    break;
    case 'postodlcs':
    postOdlcs(cookieobj.cookie, odlcs_1);
  
    break;
    case 'telemetry':
    telemetry(cookieobj.cookie,Data);
  
    break;
    case'getOdlcs':
    getOdlcs(cookieobj.cookie);
    
    break;
    case 'putOdlcs':
    putOdlcs(cookieobj.cookie,alphabet);
    
    break;
    case'exit':
    break;
    default:
    console.log('went to default... pl try again \n');
}
*/
postOdlcs(cookieobj.cookie, odlcs_1);

}
gc();

module.exports = { variableName: "variableValue" };
setTimeout(() => (switchTest()), 2000);

