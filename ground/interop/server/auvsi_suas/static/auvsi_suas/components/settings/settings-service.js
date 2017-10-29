/**
 * @fileoverview Service for settings.
 */


/**
 * Service containing app settings.
 * @final
 * @constructor
 * @struct
 * @ngInject
 */
Settings = function() {
    /*
     * @export @const {!Number} The max distance to satisfy a waypoint.
     */
    this.satistfied_waypoint_dist_max_ft = 50.0;
};


// Register the service.
angular.module('auvsiSuasApp').service('Settings', [
    Settings
]);
