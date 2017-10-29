/**
 * Tests for the Distance service.
 */


describe("Distance service", function() {
    var distance;

    beforeEach(module('auvsiSuasApp'));

    beforeEach(inject(function(Distance) {
        distance = Distance;
    }));

    var distanceAboutEqual = function(dist, expect_dist) {
        var diff = Math.abs(dist - expect_dist);
        expect(diff).toBeLessThan(10);
    };

    it("Should calculate haversine", function() {
        d = [
            // Lon1, lat1, lon2, lat2, distance
            [  0,         0,          0,         0,           0],
            [  1,         1,          1,         1,           0],
            [-76,        42,        -76,        42,           0],
            [-76.428709, 38.145306, -76.426375, 38.146146,  736.4],
            [-76.428537, 38.145399, -76.427818, 38.144686,  329.6],
            [-76.434261, 38.142471, -76.418876, 38.147838, 4820.0]
        ]

        for (var i = 0; i < d.length; i++) {
            dist = distance.haversine(d[i][1], d[i][0], d[i][3], d[i][2]);
            distanceAboutEqual(dist, d[i][4]);
        }
    });

    it("Should calculate distance", function() {
        d = [
            // Lon1, lat1, alt1, lon2, lat2, alt2, distance
            [  0,         0,          0,    0,         0,          0,     0],
            [  1,         2,          3,    1,         2,          3,     0],
            [-30,        30,        100,  -30,        30,        100,     0],
            [-76.428709, 38.145306,   0,  -76.426375, 38.146146,   0,   736.4],
            [-76.428537, 38.145399,   0,  -76.427818, 38.144686, 100,   344.4],
            [-76.434261, 38.142471, 100,  -76.418876, 38.147838, 800,  4873.7]
        ]

        for (var i = 0; i < d.length; i++) {
            dist = distance.distanceTo(d[i][1], d[i][0], d[i][2], d[i][4],
                                       d[i][3], d[i][5]);
            distanceAboutEqual(dist, d[i][6]);
        }
    });
});
