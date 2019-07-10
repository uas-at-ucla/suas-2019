import React from "react";
import ReactDOM from "react-dom";
import { Provider } from "react-redux";

import "bootstrap/dist/css/bootstrap.min.css";
import "font-awesome/css/font-awesome.min.css";
import "sanitize.css";

import registerServiceWorker from "./registerServiceWorker";

import App from "./components/App";
import "./index.css";
import store from "./redux/store";

import { loadGoogleMapsApi } from "./components/utils/GoogleMap/google";

// declare Node.js properties of 'window' provided by Electron
declare global {
  interface Window {
    require: typeof require;
    process: typeof process;
  }
}

loadGoogleMapsApi(() => {
  ReactDOM.render(
    <Provider store={store}>
      <App />
    </Provider>,
    document.getElementById("root")
  );
});
registerServiceWorker();
