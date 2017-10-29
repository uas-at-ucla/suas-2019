/**
 * @fileoverview Directive for 3D map view of Mission Scene.
 */


/**
 * Directive for 3D map view of Mission Scene.
 * @param {!angular.Window} $window The window service.
 * @param {!angular.Scope} $rootScope The root scope service.
 * @param {!Object} Distance The distance service.
 * @param {!Object} Units The units service.
 * @final
 * @constructor
 * @struct
 * @ngInject
 */
MissionMapCtrl = function($window, $rootScope, Distance, Units, Settings) {
    /**
     * @private @const {!angular.Window} The window service.
     */
    this.window_ = $window;

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
     * @private @const {!Object} The mission scene, created on link.
     */
    this.missionScene_ = null;

    /**
     * @private {?angular.Scope} The element scope.
     */
    this.scope_ = null;

    /**
     * @private {?Array} The directive element where the map is drawn.
     */
    this.element_ = null;

    /**
     * @private {?Object} The directive element attributes.
     */
    this.attrs_ = null;

    /**
     * @private {?number} The width of the view.
     */
    this.width_ = null;

    /**
     * @private {?number} The height of the view.
     */
    this.height_ = null;

    /**
     * @private {!number} Whether the renderer is active.
     */
    this.renderActive_ = false;

    /**
     * @private {?Object} The WebGL renderer.
     */
    this.renderer_ = null;

    /**
     * @private {?Object} The camera.
     */
    this.camera_ = null;

    /**
     * @private @const {!Object} Raycaster for view, used to handle
     *     mouse interactions.
     */
    this.raycaster_ = new THREE.Raycaster();

    /**
     * @private @const {!Object} The position on screen when mouse went down.
     */
    this.mouseDownPos_ = new THREE.Vector2();

    /**
     * @private @const {!Object} The position on screen when mouse moved.
     */
    this.mouseMovePos_ = new THREE.Vector2();

    /**
     * @private @const {!Object} The position in the world when mouse went down.
     */
    this.mouseDownWorldPos_ = new THREE.Vector3();

    /**
     * @private @const {!Object} The position in the world when mouse was moved.
     */
    this.mouseMoveWorldPos_ = new THREE.Vector3();
};


/**
 * Initialize the directive instance.
 * @param {!angular.Scope} scope The directive scope.
 * @param {!Array} element The directive element.
 * @param {!Object} attrs The directive attributes.
 */
MissionMapCtrl.prototype.link = function(scope, element, attrs) {
    // Store directive information.
    this.scope_ = scope;
    this.element_ = element;
    this.attrs_ = attrs;

    // Create the scene for this element.
    this.missionScene_ = new MissionScene(
            this.rootScope_, this.distance_, this.units_, this.settings_);
    this.scope_.missionScene = this.missionScene_;

    // Create a camera and configure it.
    var fieldOfView = 60;
    var aspectRatio = 1;
    var nearClipPlane = 0.1;
    var farClipPlane = 100000;
    this.camera_ = new THREE.PerspectiveCamera(
            fieldOfView, aspectRatio, nearClipPlane, farClipPlane);
    // Start the camera above the ground, with 45deg azimuth, looking at origin.
    this.camera_.position.set(0, -200, 1600);
    this.camera_.up = new THREE.Vector3(0, 0, 1);
    this.camera_.lookAt(new THREE.Vector3(0, 0, 0));

    // Create a renderer and configure it.
    this.renderer_ = new THREE.WebGLRenderer({antialias: true,
                                              precision: 'highp',
                                              alpha: true});
    this.renderer_.shadowMapEnabled = true;
    this.renderer_.shadowMapType = THREE.PCFSoftShadowMap;
    // Add renderer to DOM.
    element.append(this.renderer_.domElement);

    // Set camera and renderer sizes.
    this.setCameraAndRendererSize_();

    // Whenever the window resizes, update the camera and renderer.
    angular.element(this.window_).on(
            'resize', angular.bind(this, this.setCameraAndRendererSize_));

    // Whenever mouse is pressed/moved/scrolled, update camera.
    angular.element(element).on(
            'mousedown', angular.bind(this, this.mouseUpDown_, true));
    angular.element(element).on(
            'mouseup', angular.bind(this, this.mouseUpDown_, false));
    angular.element(element).on(
            'mousemove', angular.bind(this, this.mouseMoved_));
    angular.element(element).on(
            'mousewheel', angular.bind(this, this.mouseScrolled_));

    // Start render loop.
    this.renderActive_ = true;
    this.render_();

    // When directive destroyed, unlink.
    scope.$on('$destroy', angular.bind(this, this.unlink_));
};


/**
 * Stop the renderer and destroy instances.
 * @private
 */
MissionMapCtrl.prototype.unlink_ = function() {
    this.renderActive_ = false;
    this.camera_ = null;
    this.renderer_ = null;
};


/**
 * Sets the camera and renderer to match the view.
 * @private
 */
MissionMapCtrl.prototype.setCameraAndRendererSize_ = function() {
    this.width_ = this.element_[0].offsetWidth - this.scope_.offsetWidth;
    this.height_ = this.window_.innerHeight - this.scope_.offsetHeight;
    // Set renderer size.
    this.renderer_.setSize(this.width_, this.height_);
    // Set camera aspect ratio.
    this.camera_.aspect = this.width_ / this.height_;
    this.camera_.updateProjectionMatrix();
};


/**
 * Gets the normalized device coordinates for the position.
 * @param {!number} offsetX The screen X coordinate.
 * @param {!number} offsetY The screen Y coordinate.
 * @return {!Object} The normalized device coordinates.
 * @private
 */
MissionMapCtrl.prototype.getNormalizedDeviceCoords_ = function(
        offsetX, offsetY, pos) {
    var pos = new THREE.Vector2();
    pos.x = (offsetX / this.width_) * 2 - 1;
    pos.y = (offsetY / this.height_) * -2 + 1;
    return pos;
};


/**
 * Gets the mouse world position for a click on the ground.
 * @param {!Object} mousePos The mouse position.
 * @return {?Object} The mouse world position, or null if no
 *     intersection was found.
 * @private
 */
MissionMapCtrl.prototype.getMousePositionOnGround_ = function(
        mousePos, mouseWorldPos) {
    this.raycaster_.setFromCamera(mousePos, this.camera_);
    var intersects = this.raycaster_.intersectObject(this.missionScene_.ground);
    if (intersects.length == 0) {
        return null;
    }
    return intersects[0].point;
};


/**
 * Updates the state of the mouse click.
 * @param {!boolean} mouseDown Whether the mouse is down.
 * @param {!Object} event THe mouse event.
 * @private
 */
MissionMapCtrl.prototype.mouseUpDown_ = function(mouseDown, event) {
    this.mouseDown_ = mouseDown;

    if(mouseDown) {
        // Track the mouse down position in world coordinates.
        this.mouseDownPos_ = this.getNormalizedDeviceCoords_(
                event.offsetX, event.offsetY);
        this.mouseDownWorldPos_ = this.getMousePositionOnGround_(
                this.mouseDownPos_);
    }
};


/**
 * Updates the mouse position and camera.
 * @param {!Object} event The event containing mouse details.
 * @private
 */
MissionMapCtrl.prototype.mouseMoved_ = function(event) {
    event.preventDefault();
    // If mouse is not down, ignore.
    if (!this.mouseDown_) {
        return;
    }

    // Get mouse position in world coordinates to compare against mouse down.
    this.mouseMovePos_ = this.getNormalizedDeviceCoords_(
            event.offsetX, event.offsetY);
    this.mouseMoveWorldPos_ = this.getMousePositionOnGround_(
            this.mouseMovePos_);

    // Apply offset to move position under mouse back to position at mouse down.
    if (!!this.mouseDownWorldPos_ && !!this.mouseMoveWorldPos_) {
        var mouseWorldMovement = new THREE.Vector3();
        mouseWorldMovement.subVectors(this.mouseDownWorldPos_,
                                      this.mouseMoveWorldPos_);
        this.camera_.position.x += mouseWorldMovement.x;
        this.camera_.position.y += mouseWorldMovement.y;
        this.camera_.updateMatrixWorld();
    }
};


/**
 * Updates the mouse scroll and camera.
 * @param {!Object} event The event containing mouse details.
 * @private
 */
MissionMapCtrl.prototype.mouseScrolled_ = function(event) {
    this.camera_.position.z += event.originalEvent.deltaY;
    if (this.camera_.position.z < 1) {
        this.camera_.position.z = 1;
    }
    if (this.camera_.position.z > 10000) {
        this.camera_.position.z = 10000;
    }
};


/**
 * Renders the mission scene in a render loop.
 * @private
 */
MissionMapCtrl.prototype.render_ = function() {
    if (!!this.missionScene_.scene && !!this.camera_) {
        this.renderer_.render(this.missionScene_.scene, this.camera_);
    }

    if (this.renderActive_) {
        requestAnimationFrame(angular.bind(this, this.render_));
    }
};


// Register the directive.
angular.module('auvsiSuasApp').directive('missionMap', [
    '$window',
    '$rootScope',
    'Distance',
    'Units',
    'Settings',
    function($window, $rootScope, Distance, Units, Settings) {
        var mapCtrl = new MissionMapCtrl($window, $rootScope, Distance, Units, Settings);
        return {
            restrict: 'E',
            scope: {
                offsetWidth: '=',
                offsetHeight: '=',
                missionScene: '='
            },
            templateUrl: ('/static/auvsi_suas/components/' +
                          'mission-map/mission-map.html'),
            link: angular.bind(mapCtrl, mapCtrl.link)
        };
    }
]);
