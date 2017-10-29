/**
 * @fileoverview Controller for the Mission List page.
 */


/**
 * Controller for the Mission List page.
 * @param {!Object} Backend The backend service.
 * @final
 * @constructor
 * @struct
 * @ngInject
 */
MissionListCtrl = function(Backend) {
    /**
     * @private {?Array<Object>} The missions data.
     */
    this.missions_ = null;

    // Query the backend for missions.
    Backend.missionResource.query({}).$promise.then(
            angular.bind(this, this.setMissions_));
};


/**
 * Determines whether there are missions.
 * @return {!boolean} Whether there are missions.
 * @export
 */
MissionListCtrl.prototype.hasMissions = function() {
    return !!this.missions_ && this.missions_.length != 0;
};


/**
 * @return {?Array<Object>} The missions data.
 * @export
 */
MissionListCtrl.prototype.getMissions = function() {
    return this.missions_;
};


/**
 * @param {Array<Object>} missions The missions data.
 */
MissionListCtrl.prototype.setMissions_ = function(missions) {
    this.missions_ = missions;
};


// Register controller with app.
angular.module('auvsiSuasApp').controller('MissionListCtrl', [
    'Backend',
    MissionListCtrl
]);
