/**
 * Tests for the Units service.
 */


describe("Units service", function() {
    var units;

    beforeEach(module('auvsiSuasApp'));

    beforeEach(inject(function(Units) {
        units = Units;
    }));

    var approxEqual = function(val, expect_val) {
        var diff = Math.abs(val - expect_val);
        expect(diff).toBeLessThan(0.02);
    };

    it("Should convert degrees to radians", function() {
        d = [[0,   0],
             [180, Math.PI],
             [360, Math.PI * 2]];
        for (var i = 0; i < d.length; i++) {
            approxEqual(units.degToRad(d[i][0]), d[i][1]);
        }
    });

    it("Should convert kilometers to feet", function() {
        d = [[0,         0],
             [1,      3280.84],
             [1.5,    4921.26],
             [100,  328084]];
        for (var i = 0; i < d.length; i++) {
            approxEqual(units.kmToFt(d[i][0]), d[i][1]);
        }
    });
});
