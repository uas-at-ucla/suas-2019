class MapUi {
  constructor(ground_interface) {
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

class Communicator {
  // Get data from Python script that is retrieving data about the drone.

  constructor(ground_interface) {
    var SOCKET_DOMAIN = "0.0.0.0";
    var SOCKET_PORT = 5000;

    var SOCKET_ADDRESS = "http://" + SOCKET_DOMAIN + ":" + SOCKET_PORT;

    this.socket = io.connect(SOCKET_ADDRESS);

    this.socket.on('connect', function() {
      console.log("Connected to ground interface feeder!");
    });
  }
}

class GroundInterface {
  // Manage all UI subcomponents that allow the interface to run.

  constructor() {
    this.communicator = new Communicator(this);

    this.map_ui = new MapUi(this);
  }
}

$(document).ready(function() {
  var ground_interface = new GroundInterface();
});
