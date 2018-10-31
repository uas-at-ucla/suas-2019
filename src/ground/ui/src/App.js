import React, { Component } from 'react';
import { BrowserRouter as Router, Route, Link } from "react-router-dom";

import './App.css';
import Controls from './components/Controls/Controls';
import Vision from './components/Vision/Vision';
import Settings from './components/Settings/Settings';

class App extends Component {
  render() {
    return (
      <div className="App">
        <Router>
          <div>
            <nav>
              <ul>
                <li>
                  <Link to="/">Controls</Link>
                </li>
                <li>
                  <Link to="/vision/">Vision</Link>
                </li>
                <li>
                  <Link to="/settings/">Settings</Link>
                </li>
              </ul>
            </nav>

            <Route path="/" exact component={Controls} />
            <Route path="/vision/" component={Vision} />
            <Route path="/settings/" component={Settings} />
          </div>
        </Router>
      </div>
    );
  }
}

export default App;
