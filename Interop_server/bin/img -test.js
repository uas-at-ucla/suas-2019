var fs = require('fs');
var jpeg = require('jpeg-js');
var jpegData = fs.readFileSync('./2x2.jpg');
var rawImageData = jpeg.decode(jpegData); // return as Uint8Array
console.log(rawImageData);