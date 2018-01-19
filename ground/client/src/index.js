import React from "react";
import ReactDOM from "react-dom";
import "./index.css";

import "bootstrap/dist/css/bootstrap.css";

import App from "./App";
import registerServiceWorker from "./registerServiceWorker";

import "font-awesome/css/font-awesome.min.css";
import "bootstrap/dist/css/bootstrap.min.css";

import jquery from "jquery";
import Popper from "popper.js/dist/esm/popper.min.js";

import tether from "react-tether";

window.$ = jquery;
window.jQuery = jquery;
window.Popper = Popper;
window.Tether = tether;

require("bootstrap/dist/js/bootstrap.min.js");

ReactDOM.render(<App />, document.getElementById("root"));
registerServiceWorker();

window.onbeforeunload = function() {
  return true;
};
// Comment out last line for test flights
window.onbeforeunload = null;

