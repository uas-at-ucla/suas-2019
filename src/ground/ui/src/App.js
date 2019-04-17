import React, { Component } from 'react';
import { BrowserRouter as Router, Route, Link, Redirect, Switch } from "react-router-dom";

import './App.css';
import Controls from './components/Controls/Controls';
import Vision from './components/Vision/Vision';
import Analytics from './components/Analytics/Analytics';
import Settings from './components/Settings/Settings';

const defaultPage = "/controls" // "/vision"

class App extends Component {
  render() {
    return (
      <div>
        <Router>
          <div className="routerDiv">
            <nav className="viewNav">
              <ul>
                <li>
                  <Link to="/controls"> Controls </Link>
                </li>
                <li>
                  <Link to="/vision"> Vision </Link>
                </li>
                <li>
                  <Link to="/analytics"> Analytics </Link>
                </li>
                <li>
                  <Link to="/settings"> Settings </Link>
                </li>
              </ul>
            </nav>

            <Switch>
              <Route path="/controls" component={Controls} />
              <Route path="/vision" component={Vision} />
              <Route path="/analytics" component={Analytics} />
              <Route path="/settings" component={Settings} />
              <Redirect to={defaultPage} />
            </Switch>
          </div>
        </Router>
      </div>
    );
  }
}

export default App;
