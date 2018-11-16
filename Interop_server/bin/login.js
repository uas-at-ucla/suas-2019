#! /usr/bin/env node
var Cookie_1 = "isjh";
const axios = require('axios');
//var readline = require('readline-sync');
var coockiePlus = "";
let a = "";
var cookie;
    export function gc (Cookie_1){axios ({
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
         response.cookie=  test.substring(i+3, j);
      
        
        Cookie_1 = JSON.stringify(response.cookie);
     
        
    })
    .then(function lol () {
        console.log(Cookie_1)
        return Cookie_1;
    })
    .catch(function (error) {
        console.log(error);
   
   
    })
}
function c (a,gc,Cookie_1)
{
    a = gc(Cookie_1)
    console.log(a) 
    return a;
}

function sleep(milliseconds) {
    var start = new Date().getTime();
    for (var i = 0; i < 1e7; i++) {
      if ((new Date().getTime() - start) > milliseconds){
        break;
      }
    }
}
sleep(1100000)
var b = c(a, gc, Cookie_1)
console.log(b)
sleep(100000)
var d = gc(Cookie_1)
console.log(d)