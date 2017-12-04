import React from 'react';
import ReactDOM from 'react-dom';
import './index.css';
import App from './App';
import registerServiceWorker from './registerServiceWorker';

import 'font-awesome/css/font-awesome.min.css';
import 'bootstrap/dist/css/bootstrap.min.css';

import $ from 'jquery/dist/jquery.min.js';
import Popper from 'popper.js/dist/esm/popper.min.js';

window.$ = $;
window.Popper = Popper;
require('bootstrap/dist/js/bootstrap.min.js');

ReactDOM.render(<App />, document.getElementById('root'));
registerServiceWorker();

window.onbeforeunload = function() {
    return true;
};
// Comment out last line for test flights
window.onbeforeunload = null;