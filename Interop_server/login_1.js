const axios = require('axios');

/*var instance = axios.create({
  baseURL: '127.0.0.1:8000/api',
  timeout: 1000,
  headers: {'X-Custom-Header': 'foobar'}
 });
 axios.post('/login', {
 "username" : "testadmin",
 "password" : "testpass"
})
.then(function (response) {
  console.log(response);
})
.catch(function (error) {
  console.log(error);
});*/
axios({
  method: 'post',
  baseURL: "localhost:8000/api/",
  url: '/login',
 data: {
    "username": "testadmin",
    "password": "testpass"
  },
  timeout: 5000
})
.then(function (response) {
  console.log(response);
})
.catch(function (error) {
  console.log(error);
});