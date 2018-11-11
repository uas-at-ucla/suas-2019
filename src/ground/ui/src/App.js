import React, { Component } from 'react';
import { BrowserRouter as Router, Route, Link, Redirect } from "react-router-dom";

import './App.css';
import Controls from './components/Controls/Controls';
import Vision from './components/Vision/Vision';
import Analytics from './components/Analytics/Analytics';
import Settings from './components/Settings/Settings';

const defaultPage = "/controls" // "/vision"

class App extends Component {
  render() {
    return (
      <Router>
        <div>
          <nav>
            <ul>
              <li>
                <Link to="/controls">CONTROLS</Link>
              </li>
              <li>
                <Link to="/vision">VISION</Link>
              </li>
              <li>
                <Link to="/analytics">ANALYTICS</Link>
              </li>
              <li>
                <Link to="/settings">SETTINGS</Link>
              </li>
            </ul>
          </nav>

          <Redirect exact from="/" to={defaultPage}/>
          <Route path="/controls" component={Controls} />
          <Route path="/vision" component={Vision} />
          <Route path="/analytics" component={Analytics} />
          <Route path="/settings" component={Settings} />
        </div>
      </Router>
    );
  }
}

export default App;
