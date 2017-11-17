var METERS_PER_FOOT = 0.3048;

class MapUi {
  constructor(ground_interface) {
    var field = {lat : 38.1470000, lng : -76.4284722};

    this.map = new google.maps.Map(document.getElementById('map'), {
      center : field,
      zoom : 16,
      tilt : 0,
      disableDefaultUI : true,
      scrollwheel : true,
      navigationControl : false,
      mapTypeControl : false,
      scaleControl : true,
      draggable : true,
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
      maxZoom: 16,
      minZoom: 0
    }));

    this.map.setMapTypeId("offline_gmap");

    $('#map').resize(() => {
      var center = this.map.getCenter();
      google.maps.event.trigger(this.map, "resize");
      this.map.panTo(center);
    });

    this.drone_marker_icon = {
      path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
      scale: 10,
      rotation: 0,
      anchor: new google.maps.Point(0, 2.5)
    }
    this.drone_marker = new google.maps.Marker({
      map: this.map,
      position: field,
      icon: this.drone_marker_icon
    });
    this.drone_background_marker = new google.maps.Marker({
      map: this.map,
      position: field,
      icon: {
        url: "/css/drone_marker.svg",
        anchor: new google.maps.Point(50, 50)
      }
    });

    this.stationary_obstacle_markers = [];
    this.stationary_obstacle_color = '#FF0000';
    this.moving_obstacle_markers = [];
    this.moving_obstacle_color = '#FFA500';

    var self = this;

    this.follow_drone = true;
    $('#follow_drone_btn').click(() => {
      this.follow_drone = true;
      self.pan_to_drone();
    });
    this.map.addListener('dragstart', function() {
      this.follow_drone = false;
    });

    // Always keep drone in the center of view, even after resizing.
    google.maps.event.addListener(self.map, 'bounds_changed', function() {
      if (this.follow_drone) {
        self.pan_to_drone();
      }
    });
  }

  update_drone_position(new_lat, new_lng, new_heading) {
    var new_position = new google.maps.LatLng(new_lat, new_lng);

    this.drone_marker.setPosition(new_position);
    this.drone_marker_icon.rotation = new_heading;
    this.drone_marker.setIcon(this.drone_marker_icon);
    this.drone_background_marker.setPosition(new_position);

    if(this.get_distance(new_position, this.map.getCenter()) > 50.0) {
      if (this.follow_drone) {
        this.pan_to_drone();
      }
    }
  }

  pan_to_drone() {
    this.map.panTo(this.drone_marker.getPosition());
  }

  draw_boundary(boundary_pts) {
    var boundary_coordinates = []
    for (var pt of boundary_pts) {
      boundary_coordinates.push({lat: pt.latitude, lng: pt.longitude});
    }
    var first_pt = boundary_pts[0];
    boundary_coordinates.push({lat: first_pt.latitude, lng: first_pt.longitude});
    var boundary = new google.maps.Polyline({
      path: boundary_coordinates,
      geodesic: true,
      strokeColor: '#FF0000',
      strokeOpacity: 0.7,
      strokeWeight: 3,
    });
    boundary.setMap(this.map);
  }

  set_stationary_obstacles(obstacles) {
    for (let marker of this.stationary_obstacle_markers) {
      marker.marker.setMap(null);
      marker.circle.setMap(null);
    }
    this.stationary_obstacle_markers.length = 0;
    for (let obstacle of obstacles) {
      let marker = this.make_obstacle_marker(obstacle, 
        this.stationary_obstacle_color);
      this.stationary_obstacle_markers.push(marker);
    }
  }

  set_moving_obstacles(obstacles) {
    for (let marker of this.moving_obstacle_markers) {
      marker.marker.setMap(null);
      marker.circle.setMap(null);
    }
    this.moving_obstacle_markers.length = 0;
    for (let obstacle of obstacles) {
      let marker = this.make_obstacle_marker(obstacle, 
        this.moving_obstacle_color);
      this.moving_obstacle_markers.push(marker);
    }
  }

  make_obstacle_marker(obstacle, color) {
    let pos = {lat: obstacle.latitude, lng: obstacle.longitude};
    let radius_feet = obstacle.cylinder_radius || obstacle.sphere_radius
    let marker = new google.maps.Marker({
      position: pos,
      map: this.map,
      opacity: 0.4,
      icon: {
        path: google.maps.SymbolPath.CIRCLE,
        scale: 6
      }
    });
    marker.addListener('mouseover', function() {
      marker.setOpacity(1);
    });
    marker.addListener('mouseout', function() {
      marker.setOpacity(0.4);
    });
    let infowindow = new google.maps.InfoWindow({
      content: 'Lat: ' + obstacle.latitude + '<br>' + 
               'Lng: ' + obstacle.longitude + '<br>' +
               'Radius: ' + radius_feet + ' ft'
    });
    marker.addListener('click', function() {
      infowindow.open(this.map, marker);
    });
    google.maps.event.addListener(this.map, "click", function(event) {
      infowindow.close();
    });
    let circle = new google.maps.Circle({
      fillColor: color,
      fillOpacity: 0.7,
      strokeWeight: 2,
      map: this.map,
      radius: radius_feet * METERS_PER_FOOT
    });
    circle.bindTo('center', marker, 'position');
    return {
      marker: marker,
      circle: circle,
      infowindow: infowindow,
      obstacle: obstacle
    };
  }

  update_moving_obstacles(obstacles) {
    if (obstacles.length !== this.moving_obstacle_markers.length) {
      return false;
    }
    for (let i = 0; i < obstacles.length; i++) {
      let marker = this.moving_obstacle_markers[i];
      if (obstacles[i].sphere_radius !== marker.obstacle.sphere_radius)
        return false;
      marker.marker.setPosition({
        lat: obstacles[i].latitude, lng: obstacles[i].longitude
      });
      marker.infowindow.setContent(
        'Lat: ' + obstacles[i].latitude + '<br>' + 
        'Lng: ' + obstacles[i].longitude + '<br>' +
        'Radius: ' + obstacles[i].sphere_radius + ' ft'
      );
    }
    return true;
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
      $("#armed_indicator").text("Online");
      $("#state_indicator").text("");
    });

    this.socket.on('disconnect', function() {
      console.log("Disconnected from ground interface feeder!");
        $("#armed_indicator").text("Offline");
        $("#state_indicator").text("");
    });

    var self = this;
    this.socket.on('telemetry', function(telemetry) {
      console.log("got something!");
      ground_interface.map_ui.update_drone_position(
          Number(telemetry["gps_lat"]),
          Number(telemetry["gps_lng"]),
          Number(telemetry["heading"]));

      if(telemetry["armed"] == "False") {
        $("#armed_indicator").text("Disarmed");
      } else {
        $("#armed_indicator").text("Armed");
      }

      $("#state_indicator").text(
          self.convert_to_title_text(telemetry["state"]));

      console.log(telemetry);
      var METERS_PER_SECOND_TO_MPH = 2.23694;
      $("#telemetry_speed").text(
          self.round(telemetry["air_speed"] * METERS_PER_SECOND_TO_MPH, 1)
            + "mph");
      $("#telemetry_altitude").text(
          self.round(telemetry["gps_rel_alt"], 1) + " meters");
      $("#telemetry_position").text(
          self.round(telemetry["gps_lat"], 7) + ", "
          + self.round(telemetry["gps_lng"], 7));
      $("#telemetry_satellites").text(
          self.round(telemetry["gps_satellites"], 7));
      $("#telemetry_heading").text(
          self.round(telemetry["heading"], 7));
    });

    this.socket.on('missions_and_obstacles', function(data) {
      ground_interface.map_ui.set_stationary_obstacles(
        data.stationary_obstacles);
      ground_interface.map_ui.set_moving_obstacles(data.moving_obstacles);
      var mission = data.missions[0];
      for (var fly_zone of mission.fly_zones) {
        ground_interface.map_ui.draw_boundary(fly_zone.boundary_pts);
      }
    });

    this.socket.on('moving_obstacles', function(moving_obstacles) {
      if (!ground_interface.map_ui.update_moving_obstacles(moving_obstacles)) {
        console.log("Different moving obstacles received.");
        ground_interface.map_ui.set_moving_obstacles(moving_obstacles);
      }
    });
  }

  convert_to_title_text(str) {
    return str.replace(/\w\S*/g, function(txt){
      return txt.charAt(0).toUpperCase() + txt.substr(1).toLowerCase();
    });
  }

  round(value, precision) {
    var multiplier = Math.pow(10, precision || 0);
    return Math.round(value * multiplier) / multiplier;
  }
}

class BasicUi {
  constructor() {
    $('#sidebar').on('transitionend', (event) => {
      if (event.target.id === 'sidebar') {
        $('#map').trigger('resize');
      }
    });
    $('#show_sidebar_btn').click(() => {
      $('#sidebar').removeClass('hidden');
      $('#sidebar_container').removeClass('hidden');
      $('#show_sidebar_btn').addClass('hidden');
    });
    $('#hide_sidebar_btn').click(() => {
      $('#sidebar').addClass('hidden');
      $('#sidebar_container').addClass('hidden');
      $('#show_sidebar_btn').removeClass('hidden');
    });
  }
}

class GroundInterface {
  // Manage all UI subcomponents that allow the interface to run.

  constructor() {
    this.basic_ui = new BasicUi(this);
    this.map_ui = new MapUi(this);
    this.communicator = new Communicator(this);
  }
}

$(document).ready(function() {
  var ground_interface = new GroundInterface();
});
