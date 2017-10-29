/**
 * Tests for the MissionListCtrl controller.
 */


describe('MissionListCtrl controller', function() {
    var httpBackend, missionListCtrl;

    beforeEach(module('auvsiSuasApp'));

    beforeEach(inject(function($httpBackend, $controller, Backend) {
        httpBackend = $httpBackend;
        missionListCtrl = $controller('MissionListCtrl', {
            'Backend': Backend
        });
    }));

    it('Shouldnt have missions with null list', function() {
        expect(missionListCtrl.hasMissions()).toBe(false);
        expect(missionListCtrl.getMissions()).toBe(null);
    });

    it('Shouldnt have missions with empty list', function() {
        httpBackend.whenGET('/api/missions').respond([]);
        httpBackend.flush();
        expect(missionListCtrl.hasMissions()).toBe(false);
        expect(missionListCtrl.getMissions().length).toEqual(0);
    });

    it('Should have missions with non-empty list', function() {
        var missions = [{id: 1}];
        httpBackend.whenGET('/api/missions').respond(missions);
        httpBackend.flush();
        expect(missionListCtrl.hasMissions()).toBe(true);
        expect(missionListCtrl.getMissions().length).toEqual(1);
        expect(missionListCtrl.getMissions()[0].toJSON()).toEqual(missions[0]);
    });
});
