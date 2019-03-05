import React from 'react';
import ReactDOM from 'react-dom';
import registerServiceWorker from './registerServiceWorker';

import 'font-awesome/css/font-awesome.min.css';
import 'bootstrap/dist/css/bootstrap.min.css';

import 'google_maps_js_api';

import './index.css';
import App from './App';

import { Provider } from 'react-redux';
import store from './store';

ReactDOM.render(<Provider store={store}><App/></Provider>, document.getElementById('root'));
registerServiceWorker();
