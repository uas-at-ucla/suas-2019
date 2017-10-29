/**
 * Controller for the Odlc Review page.
 */


/**
 * Controller for the Odlc Review page.
 * @param {!angular.Window} $window The window service.
 * @param {!Object} Backend The backend service.
 * @final
 * @constructor
 * @struct
 * @ngInject
 */
OdlcReviewCtrl = function($window, Backend) {
    /**
     * @private @const {!angular.Window} The window service.
     */
    this.window_ = $window;

    /**
     * @private{?Array<Object>} The odlcs for review.
     */
    this.odlcs_ = null;

    /**
     * @private {?Object} The odlc under review.
     */
    this.odlc_ = null;

    /**
     * @private {?Array<Object>} The odlc review details.
     */
    this.odlcDetails_ = null;

    // Query the backend for odlcs to review.
    Backend.odlcReviewResource.query({}).$promise.then(
            angular.bind(this, this.setOdlcs_));
};


/**
 * Gets the odlcs for review.
 * @return {?Array<Object>} The odlcs.
 * @export
 */
OdlcReviewCtrl.prototype.getReviewOdlcs = function() {
    return this.odlcs_;
};


/**
 * @return {?Object} The odlc under review.
 */
OdlcReviewCtrl.prototype.getReviewOdlc = function() {
    return this.odlc_;
};


/**
 * @return {?Object} The odlc details.
 */
OdlcReviewCtrl.prototype.getReviewOdlcDetails = function() {
    return this.odlcDetails_;
};


/**
 * Gets the class for the odlc.
 * @param {!Object} odlc The odlc.
 * @return {string} The class.
 * @export
 */
OdlcReviewCtrl.prototype.getOdlcButtonClass = function(odlc) {
    var thumbnailClass;
    if (odlc.thumbnail_approved == null) {
        thumbnailClass = 'button';
    } else if (odlc.thumbnail_approved) {
        thumbnailClass = 'success button';
    } else {
        thumbnailClass = 'alert button';
    }

    if (this.odlc_ && odlc.id == this.odlc_.id) {
        return thumbnailClass + ' disabled';
    } else {
        return thumbnailClass;
    }
};


/**
 * Sets the odlc under review.
 * @param {!Object} odlc The odlc to review.
 * @export
 */
OdlcReviewCtrl.prototype.setReviewOdlc = function(odlc) {
    this.odlc_ = odlc;

    if (this.odlc_ == null) {
        return;
    }

    this.odlcDetails_ = [
        {'key': 'ID',
         'value': this.odlc_.id},
        {'key': 'Type',
         'value': this.odlc_.type},
    ];
    if (this.odlc_.type == 'standard' || this.odlc_.type == 'off_axis') {
        this.odlcDetails_ = this.odlcDetails_.concat([
            {'key': 'Alpha Color',
             'value': this.odlc_.alphanumeric_color},
            {'key': 'Alpha',
             'value': this.odlc_.alphanumeric},
            {'key': 'Shape Color',
             'value': this.odlc_.background_color},
            {'key': 'Shape',
             'value': this.odlc_.shape}
        ]);
    } else {
        this.odlcDetails_ = this.odlcDetails_.concat([
            {'key': 'Desc',
             'value': this.odlc_.description}
        ]);
    }
};


/**
 * Saves the review status for the odlc under review. Advances to
 * the next odlc for review.
 */
OdlcReviewCtrl.prototype.saveReview = function() {
    this.odlc_.$put().then(
            angular.bind(this, this.nextOdlc_));
};


/**
 * @return {string} The class info for the odlc image review.
 */
OdlcReviewCtrl.prototype.getOdlcImgStyle = function() {
    if (!!this.odlc_) {
        return 'background-image: url(/api/odlcs/' + this.odlc_.id +
                '/image); height: ' + this.getOdlcImgHeight() + 'px;';
    } else {
        return '';
    }
};


/**
 * @return {number} The height of the odlc image display.
 */
OdlcReviewCtrl.prototype.getOdlcImgHeight = function() {
    return this.window_.innerHeight - 95;
};


/**
 * @param {Array<Object>} odlcs The odlcs to review.
 */
OdlcReviewCtrl.prototype.setOdlcs_ = function(odlcs) {
    this.odlcs_ = odlcs;
    if (this.odlcs_.length > 0) {
        this.setReviewOdlc(this.odlcs_[0]);
    } else {
        this.setReviewOdlc(null);
    }
};


/**
 * Advances to the next odlc.
 */
OdlcReviewCtrl.prototype.nextOdlc_ = function() {
    var odlcs = this.getReviewOdlcs();

    for (var i = 0; i < odlcs.length-1; i++) {
        if (odlcs[i].id == this.odlc_.id) {
            this.setReviewOdlc(odlcs[i+1]);
            return;
        }
    }

    this.setReviewOdlc(odlcs[odlcs.length-1]);
};


// Register controller with app.
angular.module('auvsiSuasApp').controller('OdlcReviewCtrl', [
    '$window',
    'Backend',
    OdlcReviewCtrl
]);
