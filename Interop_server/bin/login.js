#! /usr/bin/env node
var cookie = "false";
var cookieobj = {};
const axios = require('axios');


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
gc()


module.exports = { variableName: "variableValue" };
setTimeout(() => console.log(cookieobj.cookie), 2000);
setTimeout(() => (module.exports.cookie= cookieobj.cookie), 2001);
