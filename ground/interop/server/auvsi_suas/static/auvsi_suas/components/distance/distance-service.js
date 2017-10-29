/**
 * @fileoverview Service to perform distance calculations.
 */


/**
 * Service to perform distance calculations.
 * @param {!Object} Units The units service.
 * @final
 * @constructor
 * @struct
 * @ngInject
 */
Distance = function(Units) {
    /**
     * @private @const {!Object} The units service.
     */
    this.units_ = Units;
};


/**
 * Calculates the haversine (distance) of two positions.
 * @param {!number} lat1 The first latitude position.
 * @param {!number} lon1 The first longitude position.
 * @param {!number} lat2 The second latitude position.
 * @param {!number} lon2 The second longitude position.
 * @return {!number} The distance in feet.
 */
Distance.prototype.haversine = function(lat1, lon1, lat2, lon2) {
    lat1 = this.units_.degToRad(lat1);
    lon1 = this.units_.degToRad(lon1);
    lat2 = this.units_.degToRad(lat2);
    lon2 = this.units_.degToRad(lon2);

    var dlon = lon2 - lon1;
    var dlat = lat2 - lat1;
    var havA = (Math.pow(Math.sin(dlat/2), 2) +
                (Math.cos(lat1) * Math.cos(lat2) *
                 Math.pow(Math.sin(dlon/2), 2)));
    var havC = 2 * Math.asin(Math.sqrt(havA));
    var distKm = 6371 * havC;
    var distFt = this.units_.kmToFt(distKm);

    return distFt;
};


/**
 * Calculates the distance of two aerial positions.
 * @param lat1 {!number} The first latitude.
 * @param lon1 {!number} The first longitude.
 * @param alt1 {!number} The first altitude in feet.
 * @param lat2 {!number} The second latitude.
 * @param lon2 {!number} The second longitude.
 * @param alt2 {!number} The second altitude in feet.
 * @return {!number} The distance in feet.
 */
Distance.prototype.distanceTo = function(lat1, lon1, alt1, lat2, lon2, alt2) {
    var haversineFt = this.haversine(lat1, lon1, lat2, lon2);
    var altFt = Math.abs(alt1 - alt2);
    var distanceFt = Math.sqrt(Math.pow(haversineFt, 2) + Math.pow(altFt, 2));
    return distanceFt;
};


// Register the service.
angular.module('auvsiSuasApp').service('Distance', [
    'Units',
    Distance
]);
