class MapUi {
  constructor(ui) {
    this.ui = ui;

    this.map = new google.maps.Map(document.getElementById('map'), {
      center : {lat : 0.0, lng : 0.0},
      zoom : 3,
      tilt : 0,
      disableDefaultUI : true,
      scrollwheel : false,
      navigationControl : false,
      mapTypeControl : false,
      scaleControl : false,
      draggable : false,
      styles : map_style,
      mapTypeId : 'hybrid'
    });
  }
}

class GroundUi {
  constructor() {
    // Initialize all sub UI components.
    this.map_ui = new MapUi(this);
  }
}

$(document).ready(function() {
  var ground_ui = new GroundUi();
});
