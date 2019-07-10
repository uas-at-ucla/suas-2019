// UNCOMMENT TO USE GOOGLE MAPS OFFLINE (1/2):
import "google_maps_js_api";

export function getLocalImageUrl(coord: google.maps.Point, zoom: number) {
  // UNCOMMENT TO USE GOOGLE MAPS OFFLINE (2/2):
  return require(`google_maps_js_api/map_images/${zoom}/mag-${zoom}_x-${coord.x}_y-${coord.y}.jpg`);
  return "";
}

const googleMapsApiURL =
  "https://maps.googleapis.com/maps/api/js?v=3.exp&libraries=geometry,drawing,visualization&key=AIzaSyBI-Gz_lh3-rKXFwlpElD7pInA60U-iK0c";

const google = {} as GoogleMapsLoader.google;

export function loadGoogleMapsApi(callback: () => void) {
  if ((window as any).google) {
    google.maps = (window as any).google.maps;
    callback();
  } else {
    const script = document.createElement("script");
    script.src = googleMapsApiURL;
    document.body.appendChild(script);

    script.onload = () => {
      if ((window as any).google) {
        google.maps = (window as any).google.maps;
        if (callback) callback();
      } else {
        throw Error("Google Maps API didn't load correctly");
      }
    };
  }
}

export default google;
