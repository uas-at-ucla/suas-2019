import React from 'react';
import ReactDOM from 'react-dom';
import { Provider } from 'react-redux';

import 'sanitize.css';
import 'font-awesome/css/font-awesome.min.css';
import 'bootstrap/dist/css/bootstrap.min.css';

import registerServiceWorker from './registerServiceWorker';

import './index.css';
import App from './components/App';
import store from './redux/store';

ReactDOM.render(<Provider store={store}><App/></Provider>, document.getElementById('root'));
registerServiceWorker();
