#! /usr/bin/env node

var usrnm="testadmin";
var psswrd="testpass";
var ckie = "invalid";
var readlineSync = require('readline-sync');
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
var instance = {
    username: usrnm,
    password: psswrd,
    logedIn: false,
    takeLoginInfo: function(){
        usrnm= readlineSync.question('Username: ');
        psswrd= readlineSync.question('Password ');
        if (usrnm=="d"&&psswrd=='d')
        {
            usrnm="testadmin";
            psswrd="testpass";
        }
    },
    cookie: ckie,
    login: function ()
    {
        var check = false;
       // instance.takeLoginInfo();
        axios ({
        
        method: 'post',
        url: "localhost:8000/api/login",
        proxy: {
            host: '127.0.0.1',
            port: 8000
          },
        timeout: 5000,
        data: {
          "username": instance.username,
          "password": instance.password
        }   
    })
    .then(function then_ (response, cookie,res) {
        var test = JSON.stringify(response.headers['set-cookie']);
        var i = test.search("=");
        var j = test.search(";")
        ckie = test.substring(i, j);
        check = true;
       })
    .then()
    .catch(function catch_ (error) {
       console.log(error);
       check = false;
       
       
    console.log(check)
    return new Promise(function (resolve, reject){
       then_().then(resolve())
       catch_().catch(reject())
    })
})
}
}
var functionSet = {
    getMission: function (){
        axios.get('localhost:8000/api/missions/1', {
             proxy: {
                host: '127.0.0.1',
                port: 8000
              },
             headers:{
                 Cookie: instance.cookie
             } ,
             timeout: 5000
           })
        .then(function (response) {
        
         console.log(response.data);
        })
        .catch(function (error) {
         console.log(error);
        });
    },
    getObstacles: function() {
        axios.get('localhost:8000/api/obstacles', {
             params: {
               Cookie: instance.cookie
             },
             proxy: {
                host: '127.0.0.1',
                port: 8000
              },
             headers:{
                 Cookie: instance.cookie
             } ,
             timeout: 5000
           })
        .then(function (response) {
         
         console.log(response.data);
        })
        .catch(function (error) {
         console.log(error);
        });
    },
    postOdlcs: function(Odlcs){
        axios ({
        method: 'post',
        url: "localhost:8000/api/odlcs",
        proxy: {
            host: '127.0.0.1',
            port: 8000
          },
        headers:{
            Cookie: instance.cookie
        } ,
        timeout: 5000,
        data: Odlcs
    })
    .then(function (response) {
     
     console.log(response.data);
    })
    .catch(function (error) {
     console.log(error);
    });
    },
    telemetry: function(data){
        axios ({
            method: 'post',
            url: "localhost:8000/api/telemetry",
            proxy: {
                host: '127.0.0.1',
                port: 8000
              },
            headers:{
                Cookie: instance.cookie,
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
    },
    getOdlcs:function(){
        axios ({
            method: 'get',
            url: "localhost:8000/api/odlcs",
            proxy: {
                host: '127.0.0.1',
                port: 8000
              },
            headers:{
                Cookie: instance.cookie,
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
    },
    putOdlcs: function(alpha)
    {
        axios({
            method : 'put',
            url : 'localhost:8000/api/odlcs/1',
            proxy: {
                host: '127.0.0.1',
                port: 8000
              },
            headers:{
                Cookie: instance.cookie,
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
}

var functionsForUse = {
    getMission : function(){
        loggedIn
        .then(function (fullfilled){
            functionSet.getMission
        })
        .catch(function(error){
            console.log(error.message)
        })
    }
}

function switchTest(){
   
    var f;
    var index;
    var functions;
functions = ['getMission', 'getObstacles', 'postOdlcs','getOdlcs', 'putOdlcs','telemetry','exit']
index = readlineSync.keyInSelect(functions, 'Which function do you wanna test? ');
f= functions[index];

console.log('Ok, ' + functions[index] +' is being tested.');

switch(f)
    {
        case 'getMission':
        functionSet.getMission();
        break;

        case 'getObstacles':
        fucntionSet.getObstacles();
    
        break;
        case 'postodlcs':
        functionSet.postOdlcs(odlcs_1);
  
        break;
        case 'telemetry':
        functionSet.telemetry(Data);
  
        break;
        case'getOdlcs':
        functionSet.getOdlcs();
    
        break;
        case 'putOdlcs':
        fucntionSet.putOdlcs(alphabet);
    
        break;
        case'exit':
        break;
        default:
        console.log('went to default... pl try again \n');
    }
}
newFunction();
function newFunction() {
    instance.login().then(console.log(instance.cookie));
}

