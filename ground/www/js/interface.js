class MapUi {
  constructor(ground_interface) {
    var field = {lat : 38.1470000, lng : -76.4284722};

    this.map = new google.maps.Map(document.getElementById('map'), {
      center : field,
      zoom : 16,
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

    this.marker = new google.maps.Marker({
      map: this.map,
      position: field
    });
  }
}

class Communicator {
  // Get data from Python script that is retrieving data about the drone.

  constructor(ground_interface) {
    var SOCKET_DOMAIN = "0.0.0.0";
    var SOCKET_PORT = 8084;

    var SOCKET_ADDRESS = "http://" + SOCKET_DOMAIN + ":" + SOCKET_PORT;

    this.socket = io.connect(SOCKET_ADDRESS);

    this.socket.on('connect', function() {
      console.log("Connected to ground interface feeder!");
    });

    this.socket.on('telemetry', function(telemetry) {
      console.log("got something!");
      ground_interface.map_ui.marker.setPosition(
          {lat: Number(telemetry["gps_lat"]),
           lng: Number(telemetry["gps_lng"])});
    });
  }
}

class GroundInterface {
  // Manage all UI subcomponents that allow the interface to run.

  constructor() {
    this.map_ui = new MapUi(this);

    this.communicator = new Communicator(this);
  }
}

$(document).ready(function() {
  var ground_interface = new GroundInterface();
});
