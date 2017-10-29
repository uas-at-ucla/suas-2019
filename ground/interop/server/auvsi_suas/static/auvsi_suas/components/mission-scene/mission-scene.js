/**
 * @fileoverview Class to build 3D scenes representing missions.
 * The built scene can be accessed via public fields of the service. When the
 * scene is rebuilt it will broadcast an 'MissionScene.sceneUpdated' event.
 *
 * The world reference frame has the `home_pos` of the `MissionConfig` at
 * the origin. The `x` axis represents longitude direction, with units of
 * feet from the `home_pos`. The `y` axis represents the latitude direction,
 * with units of feet form the `home_pos`. The `z` axis represents altitude in
 * feet MSL.
 *
 * Nothing can be shared between canvases. Thus, only one renderer can use a
 * given MissionScene at a time.
 */


/**
 * Class to build 3D scenes representing missions.
 * @final
 * @constructor
 * @struct
 * @ngInject
 */
MissionScene = function($rootScope, Distance, Units, Settings) {
    /**
     * @type {?Object} The scene that is built by the service.
     */
    this.scene = null;

    /**
     * @private @const {!angular.Scope} The root scope service.
     */
    this.rootScope_ = $rootScope;

    /**
     * @private @const {!Object} The distance service.
     */
    this.distance_ = Distance;

    /**
     * @private @const {!Object} The units service.
     */
    this.units_ = Units;

    /**
     * @private @const {!Object} The settings service.
     */
    this.settings_ = Settings;

    /**
     * @private @const {!Object} The sky light in the scene.
     */
    this.skyLight_ = new THREE.HemisphereLight(0xffffff, 0xffffff, 0.4);

    /**
     * @private @const {!Object} The sun light in the scene.
     */
    this.sunLight_ = new THREE.DirectionalLight(0xffffff, 0.6);
    this.sunLight_.position.set(0, 0, 1000);
    this.sunLight_.castShadow = true;
    this.sunLight_.shadowCameraFar = 10000;
    this.sunLight_.shadowCameraLeft = -5000;
    this.sunLight_.shadowCameraRight = 5000;
    this.sunLight_.shadowCameraTop = 5000;
    this.sunLight_.shadowCameraBottom = -5000;
    this.sunLight_.shadowMapWidth = 10000;
    this.sunLight_.shadowMapHeight = 10000;
    this.sunLight_.shadowDarkness = 0.5;
    this.sunLight_.shadowCameraVisible = true;

    /**
     * @private @const {!Object} The ground plane texture.
     */
    this.groundTexture_ = THREE.ImageUtils.loadTexture(
            '/static/auvsi_suas/components/' +
            'mission-scene/img/ground.jpg');
    this.groundTexture_.wrapS = THREE.RepeatWrapping;
    this.groundTexture_.wrapT = THREE.RepeatWrapping;
    this.groundTexture_.magFilter = THREE.LinearFilter;
    this.groundTexture_.minFilter = THREE.LinearFilter;
    this.groundTexture_.repeat.set(1000, 1000);

    /**
     * @private @const {!Object} The ground plane material.
     */
    this.groundMaterial_ = new THREE.MeshPhongMaterial(
            {color: 0x526f35,
             specular: 0xffffff,
             shininess: 0,
             map: this.groundTexture_});

    /**
     * @private @const {!Object} The ground plane geometry.
     */
    this.groundGeometry_ = new THREE.PlaneBufferGeometry(1, 1);

    /**
     * @type @const {!Object} The ground plane mesh.
     */
    this.ground = new THREE.Mesh(this.groundGeometry_, this.groundMaterial_);
    this.ground.scale.set(100000, 100000, 100000);
    this.ground.receiveShadow = true;

    /**
     * @private @const {!number} The radius to draw mission components.
     */
    this.missionComponentRadius_ = 20;

    /**
     * @private @const {!Object} The mission component geometry.
     */
    this.missionComponentGeometry_ = new THREE.SphereGeometry(1, 32, 32);

    /**
     * @private @const {!Object} The mission component material.
     */
    this.missionComponentMaterial_ = new THREE.MeshPhongMaterial(
            {color: 0xffffff});

    /**
     * @private @const {!Object} The home position marker geometry.
     */
    this.homePosGeometry_ = this.missionComponentGeometry_;

    /**
     * @private @const {!Object} The home position material.
     */
    this.homePosMaterial_ = new THREE.MeshPhongMaterial({color: 0x00ff00});

    /**
     * @private @const {!Object} The fly zone line material.
     */
    this.flyZoneLineMaterial_ = new THREE.LineBasicMaterial({color: 0x00ff00});

    /**
     * @private @const {!Object} The search grid marker geometry.
     */
    this.searchGridPtGeometry_ = this.missionComponentGeometry_;

    /**
     * @private @const {!Object} The serach grid material.
     */
    this.searchGridPtMaterial_ = new THREE.MeshPhongMaterial({color: 0x00ffff});

    /**
     * @private @const {!number} The search grid line radius.
     */
    this.searchGridPtRadius_ = this.missionComponentRadius_;

    /**
     * @private @const {!Object} The search grid line material.
     */
    this.searchGridPtLineMaterial_ = new THREE.LineBasicMaterial(
            {color: 0x00ffff});

    /**
     * @private @const {!Object} The mission waypoint geometry.
     */
    this.missionWaypointGeometry_ = this.missionComponentGeometry_;

    /**
     * @private @const {!Object} The mission waypoint material.
     */
    this.missionWaypointMaterial_ = new THREE.MeshPhongMaterial(
            {color: 0x0000ff, opacity: 0.7, transparent: true});

    /**
     * @private @const {!Object} The mission waypoint line material.
     */
    this.missionWaypointLineMaterial_ = new THREE.LineBasicMaterial(
            {color: 0x0000ff});

    /**
     * @private @const {!Object} The stationary obstacle geometry.
     */
    this.stationaryObstacleGeometry_ = new THREE.CylinderGeometry(1, 1, 1, 32);

    /**
     * @private @const {!Object} The stationary obstacle material.
     */
    this.stationaryObstacleMaterial_ = new THREE.MeshPhongMaterial(
            {color: 0xff0000, opacity: 0.7, transparent: true});

    /**
     * @private @const {!Object} The moving obstacle geometry.
     */
    this.movingObstacleGeometry_ = this.missionComponentGeometry_;

    /**
     * @private @const {!Object} The moving obstacle material.
     */
    this.movingObstacleMaterial_ = this.stationaryObstacleMaterial_;

    /**
     * @private @const {!Object} The UAS telemetry geometry.
     */
    this.telemetryGeometry_ = this.missionComponentGeometry_;

    /**
     * @private @const {!number} The UAS telemetry marker radius.
     */
    this.telemetryRadius_ = 20;

    /**
     * @private @const {!Object} The UAS telemetry material.
     */
    this.telemetryMaterial_ = new THREE.MeshPhongMaterial(
            {color: 0xffff00});
};


/**
 * Rebuild the scene with the given mission data.
 * @param {!Object} mission The mission configuration.
 * @param {!Object} obstacles The obstacles data.
 * @param {!Object} telemetry The UAS telemetry data.
 */
MissionScene.prototype.rebuildScene = function(mission, obstacles, telemetry) {
    // Create fresh scene for rebuild.
    var scene = new THREE.Scene();

    // Add the base scene elements.
    this.addBaseSceneElements_(scene);

    // Build mission scene elements. Requires a mission and home position.
    if (!!mission && !!mission.home_pos) {
        this.addMissionSceneElements_(mission, scene);
        if (!!obstacles) {
            this.addObstacleSceneElements_(
                    mission, obstacles, mission.home_pos, scene);
        }
        if (!!telemetry) {
            this.addTelemetrySceneElements_(
                    mission, telemetry, mission.home_pos, scene);
        }
    }

    // Update the scene and notify others.
    this.scene = scene;
    this.rootScope_.$broadcast('MissionScene.sceneUpdated');
};


/**
 * Creates and adds the base scene elements.
 * @param {!Object} scene The scene to add elements to.
 * @private
 */
MissionScene.prototype.addBaseSceneElements_ = function(scene) {
    // Add the ground plane.
    scene.add(this.ground);

    // Add the light.
    scene.add(this.skyLight_);
    scene.add(this.sunLight_);
};


/**
 * Adds the mission elements to the scene.
 * @param {!Object} mission The mission components to add.
 * @param {!Object} scene The scene to add elements to.
 * @private
 */
MissionScene.prototype.addMissionSceneElements_ = function(mission, scene) {
    // Add home position.
    var homePos = this.createObject_(
            this.homePosGeometry_, this.homePosMaterial_,
            mission.home_pos, this.missionComponentRadius_, mission.home_pos,
            this.missionComponentRadius_, scene);

    // Add mission components to scene.
    var missionComponents = [
        mission.air_drop_pos, mission.emergent_last_known_pos,
        mission.off_axis_odlc_pos];
    for (var i = 0; i < missionComponents.length; i++) {
        var component = missionComponents[i];
        var componentObj = this.createObject_(
                this.missionComponentGeometry_, this.missionComponentMaterial_,
                component, this.missionComponentRadius_, mission.home_pos,
                this.missionComponentRadius_, scene);
    }

    // Add fly zone lines.
    for (var i = 0; i < mission.fly_zones.length; i++) {
        var flyZone = mission.fly_zones[i];

        // Sort boundary by zone order.
        flyZone.boundary_pts.sort(function(a, b) {
            return a.order - b.order;
        });

        // Create lines for fly zone boundary.
        var altitudes = [flyZone.altitude_msl_min, flyZone.altitude_msl_max];
        var altitudeGeometries = [new THREE.Geometry(), new THREE.Geometry()];
        for (var j = 0; j < flyZone.boundary_pts.length; j++) {
            var k = (j + 1) % flyZone.boundary_pts.length;
            var start = flyZone.boundary_pts[j];
            var end = flyZone.boundary_pts[k];

            for (var l = 0; l < altitudes.length; l++) {
                var alt = altitudes[l];
                var geom = altitudeGeometries[l];
                var startPt = new THREE.Vector3();
                this.setObjectPosition_(
                        start, alt, mission.home_pos, startPt);
                var endPt = new THREE.Vector3();
                this.setObjectPosition_(
                        end, alt, mission.home_pos, endPt);
                geom.vertices.push(startPt, endPt);
            }
        }
        for (var j = 0; j < altitudeGeometries.length; j++) {
            var geom = altitudeGeometries[j];
            geom.computeLineDistances();
            var boundaryLine = new THREE.Line(geom, this.flyZoneLineMaterial_);
            scene.add(boundaryLine);
        }
    }

    // Add search grid points.
    for (var i = 0; i < mission.search_grid_points.length; i++) {
        var searchPt = mission.search_grid_points[i];
        var searchPtObj = this.createObject_(
                this.searchGridPtGeometry_, this.searchGridPtMaterial_,
                searchPt, this.searchGridPtRadius_, mission.home_pos,
                this.searchGridPtRadius_, scene);
    }

    // Add lines between search grid points.
    var searchGridPtLineGeometry = new THREE.Geometry();
    for (var i = 0; i < mission.search_grid_points.length; i++) {
        var j = (i + 1) % mission.search_grid_points.length;
        var start = mission.search_grid_points[i];
        var end = mission.search_grid_points[j];

        var startPt = new THREE.Vector3();
        this.setObjectPosition_(
                start, this.searchGridPtRadius_, mission.home_pos, startPt);
        var endPt = new THREE.Vector3();
        this.setObjectPosition_(
                end, this.searchGridPtRadius_, mission.home_pos, endPt);

        searchGridPtLineGeometry.vertices.push(startPt, endPt);
    }
    searchGridPtLineGeometry.computeLineDistances();
    var searchGridPtLine= new THREE.Line(
            searchGridPtLineGeometry, this.searchGridPtLineMaterial_);
    scene.add(searchGridPtLine);


    // Add mission waypoints.
    for (var i = 0; i < mission.mission_waypoints.length; i++) {
        var waypoint = mission.mission_waypoints[i];
        var waypointObj = this.createObject_(
                this.missionWaypointGeometry_, this.missionWaypointMaterial_,
                waypoint, waypoint.altitude_msl, mission.home_pos,
                this.settings_.satistfied_waypoint_dist_max_ft, scene);
    }

    // Add lines between mission waypoints.
    var missionWaypointLineGeometry = new THREE.Geometry();
    for (var i = 0; i < mission.mission_waypoints.length - 1; i++) {
        var start = mission.mission_waypoints[i];
        var end = mission.mission_waypoints[i+1];

        var startPt = new THREE.Vector3();
        this.setObjectPosition_(
                start, start.altitude_msl, mission.home_pos, startPt);
        var endPt = new THREE.Vector3();
        this.setObjectPosition_(
                end, end.altitude_msl, mission.home_pos, endPt);

        missionWaypointLineGeometry.vertices.push(startPt, endPt);
    }
    missionWaypointLineGeometry.computeLineDistances();
    var missionWaypointLine = new THREE.Line(
            missionWaypointLineGeometry, this.missionWaypointLineMaterial_);
    scene.add(missionWaypointLine);
};


/**
 * Adds the obstacle elements to the scene.
 * @param {!Object} mission The mission components to add.
 * @param {!Object} obstacles The obstacles to add.
 * @param {!Object} refPos A reference GPS position to convert GPS to reference
 *      frame.
 * @param {!Object} scene The scene to add elements to.
 * @private
 */
MissionScene.prototype.addObstacleSceneElements_ = function(
        mission, obstacles, refPos, scene) {
    for (var i = 0; i < obstacles.stationary_obstacles.length; i++) {
        var obstacle = obstacles.stationary_obstacles[i];
        var obstacleObj = this.createObject_(
                this.stationaryObstacleGeometry_,
                this.stationaryObstacleMaterial_, obstacle,
                obstacle.cylinder_height/2, mission.home_pos, 1, scene);
        obstacleObj.scale.set(
                obstacle.cylinder_radius, obstacle.cylinder_height,
                obstacle.cylinder_radius);
        obstacleObj.rotation.set(Math.PI/2, 0, 0);
    }

    for (var i = 0; i < obstacles.moving_obstacles.length; i++) {
        var obstacle = obstacles.moving_obstacles[i];
        var obstacleObj = this.createObject_(
                this.movingObstacleGeometry_, this.movingObstacleMaterial_,
                obstacle, obstacle.altitude_msl, mission.home_pos,
                obstacle.sphere_radius, scene);
    }
};


/**
 * Adds the telemetry elements to the scene.
 * @param {!Object} mission The mission components to add.
 * @param {!Object} telemetry The telemetry to add.
 * @param {!Object} refPos A reference GPS position to convert GPS to reference
 *     frame.
 * @param {!Object} scene The scene to add elements to.
 * @private
 */
MissionScene.prototype.addTelemetrySceneElements_ = function(
        mission, telemetry, refPos, scene) {
    for (var i = 0; i < telemetry.length; i++) {
        var userTelem = telemetry[i];
        var userTelemObj = this.createObject_(
                this.telemetryGeometry_, this.telemetryMaterial_,
                userTelem, userTelem.altitude_msl, mission.home_pos,
                this.telemetryRadius_, scene);
    }
};


/**
 * Sets the position of the object to the given GPS position.
 * @param {!Object} pos The gps position with latitude and longitude fields.
 * @param {!number} alt The altitude in feet.
 * @param {!Object} refPos The reference GPS position to convert GPS to
 *     reference frame.
 * @param {!Object} objPos The object position with x,y fields.
 * @private
 */
MissionScene.prototype.setObjectPosition_ = function(pos, alt, refPos, objPos) {
    // Compute distance components in lat/lon axis.
    var distX = this.distance_.haversine(
            refPos.latitude, pos.longitude, refPos.latitude, refPos.longitude);
    var distY = this.distance_.haversine(
            pos.latitude, refPos.longitude, refPos.latitude, refPos.longitude);
    // Set the position.
    objPos.x = Math.sign(pos.longitude - refPos.longitude) * distX;
    objPos.y = Math.sign(pos.latitude- refPos.latitude) * distY;
    objPos.z = alt;
};


/**
 * Creates a scene object with standard properties.
 * @param {!Object} geometry The geometry to use.
 * @param {!Object} material The material to use.
 * @param {!Object} pos The position to use.
 * @param {!number} alt The latitude to use.
 * @param {!Object} refPos The reference position for reference frame.
 * @param {!number} scale The object scale.
 * @param {!Object} scene The scene to add the object to.
 * @return {!Object} The object that was created and added to the scene.
 * @private
 */
MissionScene.prototype.createObject_ = function(
        geometry, material, pos, alt ,refPos, scale, scene) {
    var obj = new THREE.Mesh(geometry, material);
    this.setObjectPosition_(pos, alt, refPos, obj.position);
    obj.scale.set(scale, scale, scale);
    obj.castShadow = true;
    obj.receiveShadow = true;
    scene.add(obj);
    return obj
};
