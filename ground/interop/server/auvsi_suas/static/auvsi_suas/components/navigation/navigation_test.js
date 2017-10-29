/**
 * Tests for the Navigation service.
 */


describe("Navigation service", function() {
    var routeParams, navigationCtrl;

    beforeEach(module('auvsiSuasApp'));

    beforeEach(inject(function($controller) {
        routeParams = {};
        navigationCtrl = $controller('NavigationCtrl',
                                     {$routeParams: routeParams});
    }));

    it("Shouldn't show mission links with no ID", function() {
        expect(navigationCtrl.shouldShowMissionLinks()).toBe(false);
    });

    it("Should show mission links with an ID", function() {
        routeParams.missionId = 1;
        expect(navigationCtrl.shouldShowMissionLinks()).toBe(true);
    });
});
