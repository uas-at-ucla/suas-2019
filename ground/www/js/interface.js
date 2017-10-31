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
      //mapTypeControl : false,
      scaleControl : false,
      //draggable : false,
      styles : map_style,
    });

    this.map.mapTypes.set("offline_gmap", new google.maps.ImageMapType({
      getTileUrl: function(coord, zoom) {
        return checkTileInSprites(coord, zoom) ?
          getLocalTileImgSrc(coord, zoom) :
          getGmapTileImgSrc(coord, zoom);
      },
      tileSize: new google.maps.Size(256, 256),
      name: "LocalMyGmap",
      maxZoom: 19
    }));

    this.map.setMapTypeId("offline_gmap");

    this.marker = new google.maps.Marker({
      map: this.map,
      position: field
    });
  }

  update_drone_position(new_lat, new_lng) {
    var new_position = new google.maps.LatLng(new_lat, new_lng);

    this.marker.setPosition(new_position);

    if(this.get_distance(new_position, this.map.getCenter()) > 50.0) {
      this.map.panTo(this.marker.getPosition());  // Fixes GMaps pan glitch.
    }
  }

  rad(x) {
    return x * Math.PI / 180;
  }

  get_distance(p1, p2) {
    var R = 6378137; // Earth’s mean radius in meter
    var dLat = this.rad(p2.lat() - p1.lat());
    var dLong = this.rad(p2.lng() - p1.lng());
    var a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(this.rad(p1.lat())) * Math.cos(this.rad(p2.lat())) *
      Math.sin(dLong / 2) * Math.sin(dLong / 2);
    var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    var d = R * c;

    return d; // returns the distance in meter
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

    var self = this;
    this.socket.on('telemetry', function(telemetry) {
      console.log("got something!");
      ground_interface.map_ui.update_drone_position(
          Number(telemetry["gps_lat"]), Number(telemetry["gps_lng"]));

      if(telemetry["armed"] == "False") {
        $("#armed_indicator").text("Disarmed");
      } else {
        $("#armed_indicator").text("Armed");
      }

      $("#state_indicator").text(
          self.convert_to_title_text(telemetry["state"]));
    });
  }

  convert_to_title_text(str) {
    return str.replace(/\w\S*/g, function(txt){
      return txt.charAt(0).toUpperCase() + txt.substr(1).toLowerCase();
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
