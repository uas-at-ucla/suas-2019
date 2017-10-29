/**
 * Tests for the OdlcReviewCtrl controller.
 */


describe("OdlcReviewCtrl controller", function() {
    var httpBackend, odlcs, odlcReviewCtrl;

    beforeEach(module('auvsiSuasApp'));

    beforeEach(inject(function($window, $controller, $httpBackend, Backend) {
        httpBackend = $httpBackend;
        odlcs = [
             {id: 1, thumbnail_approved: null},
             {id: 2, thumbnail_approved: true},
             {id: 3, thumbnail_approved: false}
        ];
        httpBackend.whenGET('/api/odlcs/review').respond(odlcs);

        odlcReviewCtrl = $controller('OdlcReviewCtrl', {
            '$window': $window,
            'Backend': Backend,
        });
    }));

    it("Should get review odlcs", function() {
        httpBackend.flush();
        var recv_odlcs = odlcReviewCtrl.getReviewOdlcs();
        expect(recv_odlcs.length).toEqual(odlcs.length);
        for (var i = 0; i < odlcs.length; i++) {
            expect(recv_odlcs[i].toJSON()).toEqual(odlcs[i]);
        }
    });

    it("Should get odlc button class", function() {
        odlcReviewCtrl.setReviewOdlc(odlcs[0]);
        expect(odlcReviewCtrl.getOdlcButtonClass(odlcs[0]))
                .toEqual('button disabled');
        expect(odlcReviewCtrl.getOdlcButtonClass(odlcs[1]))
                .toEqual('success button');
        expect(odlcReviewCtrl.getOdlcButtonClass(odlcs[2]))
                .toEqual('alert button');
    });

    it("Should get review odlc", function() {
        httpBackend.flush();
        var odlc = odlcReviewCtrl.getReviewOdlcs()[0];
        odlcReviewCtrl.setReviewOdlc(odlc);
        expect(odlcReviewCtrl.getReviewOdlc().toJSON()).toEqual(odlcs[0]);
    });

    it("Should update the review", function() {
        httpBackend.flush();

        odlcs[0].thumbnail_approved = true;
        httpBackend.expectPUT('/api/odlcs/review/1', odlcs[0]).respond(200, odlcs[0]);

        var odlc = odlcReviewCtrl.getReviewOdlcs()[0];
        odlcReviewCtrl.setReviewOdlc(odlc);
        odlc.thumbnail_approved = true;
        odlcReviewCtrl.saveReview();
        httpBackend.flush();
        expect(odlc.thumbnail_approved).toBe(true);
    });
});
