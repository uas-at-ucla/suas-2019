/*global google*/
import React, { Component } from 'react';
import CustomMarker from './CustomMarker';
import { Marker } from 'react-google-maps';
import GoogleMap from '../../Utils/GoogleMap/GoogleMap';
import HeatmapLayer from "react-google-maps/lib/components/visualization/HeatmapLayer";
import './Map.css';
import { Link } from "react-router-dom";

const google = window.google;

class Map extends Component {
  constructor() {
    super();

    this.state = {
      markers: [
        {
          position: {
            lat: -34.397,
            lng: 150.644,
          },
          marker_i: 0,
          data: 'first'
        },
        {
          position: {
            lat: -38,
            lng: 150.644,
          },
          marker_i: 1,
          data: 'second'
        }
      ],
      hmData: [],
      hmGradient: [
        'rgba(0, 255, 255, 0)',
        'rgba(0, 255, 255, 1)',
        'rgba(0, 191, 255, 1)',
        'rgba(0, 127, 255, 1)',
        'rgba(0, 63, 255, 1)',
        'rgba(0, 0, 255, 1)',
        'rgba(0, 0, 223, 1)',
        'rgba(0, 0, 191, 1)',
        'rgba(0, 0, 159, 1)',
        'rgba(0, 0, 127, 1)',
        'rgba(63, 0, 91, 1)',
        'rgba(127, 0, 63, 1)',
        'rgba(191, 0, 31, 1)',
        'rgba(255, 0, 0, 1)'
      ],
      hmRadius: 70, hmOpacity: 1, hmdissipating: true, hmHeatmapOn: true,
      heatMapOpen: true,
      imageArea: {
        pixelX: 0,
        pixelY: 0,
        lat: 0,
        lnt: 0,
        width: 0,
        height: 0,
      },
      hasSelection: false,
      selectionComplete: false,
    }

    setTimeout(() =>
      this.setState({
        hmData: [
          new window.google.maps.LatLng(37.782551, -122.445368),
          new window.google.maps.LatLng(37.782745, -122.444586),
          new window.google.maps.LatLng(37.782842, -122.443688),
          new window.google.maps.LatLng(37.782919, -122.442815),
          new window.google.maps.LatLng(37.782992, -122.442112),
          new window.google.maps.LatLng(37.783100, -122.441461),
          new window.google.maps.LatLng(37.783206, -122.440829),
          new window.google.maps.LatLng(37.783273, -122.440324),
          new window.google.maps.LatLng(37.783316, -122.440023),
          new window.google.maps.LatLng(37.783357, -122.439794),
          new window.google.maps.LatLng(37.783371, -122.439687),
          new window.google.maps.LatLng(37.783368, -122.439666),
          new window.google.maps.LatLng(37.783383, -122.439594),
          new window.google.maps.LatLng(37.783508, -122.439525),
          new window.google.maps.LatLng(37.783842, -122.439591),
          new window.google.maps.LatLng(37.784147, -122.439668),
          new window.google.maps.LatLng(37.784206, -122.439686),
          new window.google.maps.LatLng(37.784386, -122.439790),
          new window.google.maps.LatLng(37.784701, -122.439902),
          new window.google.maps.LatLng(37.784965, -122.439938),
          new window.google.maps.LatLng(37.785010, -122.439947),
          new window.google.maps.LatLng(37.785360, -122.439952),
          new window.google.maps.LatLng(37.785715, -122.440030),
          new window.google.maps.LatLng(37.786117, -122.440119),
          new window.google.maps.LatLng(37.786564, -122.440209),
          new window.google.maps.LatLng(37.786905, -122.440270),
          new window.google.maps.LatLng(37.786956, -122.440279),
          new window.google.maps.LatLng(37.800224, -122.433520),
          new window.google.maps.LatLng(37.800155, -122.434101),
          new window.google.maps.LatLng(37.800160, -122.434430),
          new window.google.maps.LatLng(37.800378, -122.434527),
          new window.google.maps.LatLng(37.800738, -122.434598),
          new window.google.maps.LatLng(37.800938, -122.434650),
          new window.google.maps.LatLng(37.801024, -122.434889),
          new window.google.maps.LatLng(37.800955, -122.435392),
          new window.google.maps.LatLng(37.800886, -122.435959),
          new window.google.maps.LatLng(37.800811, -122.436275),
          new window.google.maps.LatLng(37.800788, -122.436299),
          new window.google.maps.LatLng(37.800719, -122.436302),
          new window.google.maps.LatLng(37.800702, -122.436298),
          new window.google.maps.LatLng(37.800661, -122.436273),
          new window.google.maps.LatLng(37.800395, -122.436172),
          new window.google.maps.LatLng(37.800228, -122.436116),
          new window.google.maps.LatLng(37.800169, -122.436130),
          new window.google.maps.LatLng(37.800066, -122.436167),
          new window.google.maps.LatLng(37.784345, -122.422922),
          new window.google.maps.LatLng(37.784389, -122.422926),
          new window.google.maps.LatLng(37.784437, -122.422924),
          new window.google.maps.LatLng(37.784746, -122.422818),
          new window.google.maps.LatLng(37.785436, -122.422959),
          new window.google.maps.LatLng(37.786120, -122.423112),
          new window.google.maps.LatLng(37.786433, -122.423029),
          new window.google.maps.LatLng(37.786631, -122.421213),
          new window.google.maps.LatLng(37.786660, -122.421033),
          new window.google.maps.LatLng(37.786801, -122.420141),
          new window.google.maps.LatLng(37.786823, -122.420034),
          new window.google.maps.LatLng(37.786831, -122.419916),
          new window.google.maps.LatLng(37.787034, -122.418208),
          new window.google.maps.LatLng(37.787056, -122.418034),
          new window.google.maps.LatLng(37.787169, -122.417145),
          new window.google.maps.LatLng(37.787217, -122.416715),
          new window.google.maps.LatLng(37.786144, -122.416403),
          new window.google.maps.LatLng(37.785292, -122.416257),
          new window.google.maps.LatLng(37.780666, -122.390374),
          new window.google.maps.LatLng(37.780501, -122.391281),
          new window.google.maps.LatLng(37.780148, -122.392052),
          new window.google.maps.LatLng(37.780173, -122.391148),
          new window.google.maps.LatLng(37.780693, -122.390592),
          new window.google.maps.LatLng(37.781261, -122.391142),
          new window.google.maps.LatLng(37.781808, -122.391730),
          new window.google.maps.LatLng(37.782340, -122.392341),
          new window.google.maps.LatLng(37.782812, -122.393022),
          new window.google.maps.LatLng(37.783300, -122.393672),
          new window.google.maps.LatLng(37.783809, -122.394275),
          new window.google.maps.LatLng(37.784246, -122.394979),
          new window.google.maps.LatLng(37.784791, -122.395958),
          new window.google.maps.LatLng(37.785675, -122.396746),
          new window.google.maps.LatLng(37.786262, -122.395780),
          new window.google.maps.LatLng(37.786776, -122.395093),
          new window.google.maps.LatLng(37.787282, -122.394426),
          new window.google.maps.LatLng(37.787783, -122.393767),
          new window.google.maps.LatLng(37.788343, -122.393184),
          new window.google.maps.LatLng(37.788895, -122.392506),
          new window.google.maps.LatLng(37.789371, -122.391701),
          new window.google.maps.LatLng(37.789722, -122.390952),
          new window.google.maps.LatLng(37.790315, -122.390305),
          new window.google.maps.LatLng(37.790738, -122.389616),
          new window.google.maps.LatLng(37.779448, -122.438702),
          new window.google.maps.LatLng(37.779023, -122.438585),
          new window.google.maps.LatLng(37.778542, -122.438492),
          new window.google.maps.LatLng(37.778100, -122.438411),
          new window.google.maps.LatLng(37.777986, -122.438376),
          new window.google.maps.LatLng(37.777680, -122.438313),
          new window.google.maps.LatLng(37.777316, -122.438273),
          new window.google.maps.LatLng(37.777135, -122.438254),
          new window.google.maps.LatLng(37.776987, -122.438303),
          new window.google.maps.LatLng(37.776946, -122.438404),
          new window.google.maps.LatLng(37.776944, -122.438467),
          new window.google.maps.LatLng(37.776892, -122.438459),
          new window.google.maps.LatLng(37.776842, -122.438442),
          new window.google.maps.LatLng(37.776822, -122.438391),
          new window.google.maps.LatLng(37.776814, -122.438412),
          new window.google.maps.LatLng(37.776787, -122.438628),
          new window.google.maps.LatLng(37.776729, -122.438650),
          new window.google.maps.LatLng(37.776759, -122.438677),
          new window.google.maps.LatLng(37.776772, -122.438498),
          new window.google.maps.LatLng(37.776787, -122.438389),
          new window.google.maps.LatLng(37.776848, -122.438283),
          new window.google.maps.LatLng(37.776870, -122.438239),
          new window.google.maps.LatLng(37.777015, -122.438198),
          new window.google.maps.LatLng(37.777333, -122.438256),
          new window.google.maps.LatLng(37.777595, -122.438308),
          new window.google.maps.LatLng(37.777797, -122.438344),
          new window.google.maps.LatLng(37.778160, -122.438442),
          new window.google.maps.LatLng(37.778414, -122.438508),
          new window.google.maps.LatLng(37.778445, -122.438516),
          new window.google.maps.LatLng(37.778503, -122.438529),
          new window.google.maps.LatLng(37.778607, -122.438549),
          new window.google.maps.LatLng(37.778670, -122.438644),
          new window.google.maps.LatLng(37.778847, -122.438706),
          new window.google.maps.LatLng(37.779240, -122.438744),
          new window.google.maps.LatLng(37.779738, -122.438822),
          new window.google.maps.LatLng(37.780201, -122.438882),
          new window.google.maps.LatLng(37.780400, -122.438905),
          new window.google.maps.LatLng(37.780501, -122.438921),
          new window.google.maps.LatLng(37.780892, -122.438986),
          new window.google.maps.LatLng(37.781446, -122.439087),
          new window.google.maps.LatLng(37.781985, -122.439199),
          new window.google.maps.LatLng(37.782239, -122.439249),
          new window.google.maps.LatLng(37.782286, -122.439266),
          new window.google.maps.LatLng(37.797847, -122.429388),
          new window.google.maps.LatLng(37.797874, -122.429180),
          new window.google.maps.LatLng(37.797885, -122.429069),
          new window.google.maps.LatLng(37.797887, -122.429050),
          new window.google.maps.LatLng(37.797933, -122.428954),
          new window.google.maps.LatLng(34.067364, -118.442663)]
      })
      , 1000); // Temporary: give time for google maps api to download
  }

  toggleHeatmap = (props) => {
    if (this.state.hmOpacity == 0) {
      this.setState({
        hmOpacity: 1,
        hmHeatmapOn: true
      });
      return;
    }
    else {
      this.setState({
        hmOpacity: 0,
        hmHeatmapOn: false
      });
    }
  };

  heatmapOptions = (props) => {
    if (props.open) {
      return (
        <div className="buttonPanel">
          <button className="heatmap" onClick={this.toggleHeatmap}>Toggle Heatmap</button>
          <button className="heatmap">Set Radius</button>
        </div>
      );
    }
    else {
      return (
        <div className="closedButtonPanel">
          Heatmap:
              radius: {this.state.hmRadius}
          opacity: {this.state.hmOpacity}
        </div>
      );
    }
  }

  mapDblClick = (event) => {
    // event.stopPropagation();
    let hasSelection = this.state.hasSelection;
    // console.log(event.latLng.lat(), event.latLng.lng());
    let x = event.latLng.lat();
    let y = event.latLng.lng();
    if (!hasSelection) {
      this.setState({
        imageArea: { pixelX: x, pixelY: y },
        hasSelection: !hasSelection
      });
      console.log("1st", this.state.imageArea);
    } else if (hasSelection) {
      let width = x - this.state.imageArea.pixelX;
      let height = y - this.state.imageArea.pixelY;
      this.setState({
        imageArea: {...this.state.imageArea, width: width, height: height },
        hasSelection: !hasSelection, selectionComplete: true
      });
      console.log("2nd", this.state.imageArea);
    }
  }


  handleCancelSelect = (event) => {
    console.log("right click");
    this.setState({
      imageArea: {}, selectionComplete: false,
      hasSelection: false
    });
  }

  render() {
    let customMarkers;
    if (this.state.markers) {
      customMarkers = this.state.markers.map(marker => {
        console.log(marker.position)
        return (
          <CustomMarker marker={marker} />
        )
      })
    }

    console.log(customMarkers);
    return (
      <div className="VisionMap">
        {/* <div className="buttonPanel">
          <button className="heatmap" onClick={this.toggleHeatmap}>Toggle Heatmap</button>
          <button className="heatmap">Set Radius</button>

        </div> */}
        {/* <div className="tagging">Tagging</div>
        <div className="pipeline">Pipeline</div> */}
        <div className="Map"
          onClick={this.handleAreaSelect}
          onContextMenu={this.handleCancelSelect}
        >
          <GoogleMap
            defaultZoom={16}
            defaultCenter={{ lat: 37.782551, lng: - 122.445368 }}
            defaultMapTypeId="customTiles"
            defaultOptions={{
              disableDefaultUI: true,
              disableDoubleClickZoom: true,
              scaleControl: true
            }}
            onDblClick={this.mapDblClick}
          >
            {customMarkers}
            <HeatmapLayer
              options={{
                data: this.state.hmData,
                radius: this.state.hmRadius,
                opacity: this.state.hmOpacity,
                gradient: this.state.hmGradient,
                dissipating: true,
              }}
            >
            </HeatmapLayer>
          </GoogleMap>
        </div>
      </div>
    );
  }
}
export default Map;
