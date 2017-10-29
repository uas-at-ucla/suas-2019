/**
 * @fileoverview Main application definition. This sets up the angular module
 * and the routes.
 */


/**
 * @export {!angular.Module} Application module.
 */
var auvsiSuasApp = angular.module('auvsiSuasApp', [
    'ngResource',
    'ngRoute'
]);


// Configure routes to templates and controllers.
auvsiSuasApp.config(['$routeProvider', function($routeProvider) {
    $routeProvider.
        when('/', {
            templateUrl: '/static/auvsi_suas/pages/mission-list/mission-list.html',
            controller: 'MissionListCtrl',
            controllerAs: 'missionListCtrl'
        }).
        when('/mission/:missionId', {
            templateUrl: '/static/auvsi_suas/pages/mission-dashboard/mission-dashboard.html',
            controller: 'MissionDashboardCtrl',
            controllerAs: 'missionDashboardCtrl'
        }).
        when('/mission/:missionId/odlcs', {
            templateUrl: '/static/auvsi_suas/pages/odlc-review/odlc-review.html',
            controller: 'OdlcReviewCtrl',
            controllerAs: 'odlcReviewCtrl'
        }).
        when('/mission/:missionId/evaluate', {
            templateUrl: '/static/auvsi_suas/pages/evaluate-teams/evaluate-teams.html',
            controller: 'EvaluateTeamsCtrl',
            controllerAs: 'evaluateTeamsCtrl'
        }).
        otherwise({
            redirectTo: '/'
        });
}]);
