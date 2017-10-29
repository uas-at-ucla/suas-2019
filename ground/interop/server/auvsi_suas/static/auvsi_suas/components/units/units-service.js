/**
 * @fileoverview Service to perform unit calculations.
 */


/**
 * Service to perform unit calculations.
 * @final
 * @constructor
 * @struct
 */
Units = function() {
};


/**
 * Converts degrees to radians.
 * @param {!number} deg The degree to convert.
 * @return {!number} The value in radians.
 */
Units.prototype.degToRad = function(deg) {
    return deg * (Math.PI/180.0);
};


/**
 * Converts kilometers to feet.
 * @param {!number} km The distance in kilometers.
 * @return {!number} The distance in feet.
 */
Units.prototype.kmToFt = function(km) {
    return km * 3280.8399;
};


// Register the service.
angular.module('auvsiSuasApp').service('Units', [
    Units
]);
