import React, { Component } from 'react';
import { BrowserRouter as Router, Route, Link } from "react-router-dom";

import './App.css';
import Controls from './components/Controls/Controls';
import Cosmetics from './components/Cosmetics/Cosmetics';
import Vision from './components/Vision/Vision';
import Settings from './components/Settings/Settings';

class App extends Component {
  componentDidMount() {
    document.title = 'Flight Deck';
  }

  render() {
    return (
      <Router>
        <div>
          <nav>
            <ul>
              <li>
                <Link to = "/">CONTROLS</Link>
              </li>
              <li>
                <Link to = "/vision">VISION</Link>
              </li>
              <li>
                <Link to = "/settings">SETTINGS</Link>
              </li>
            </ul>
          </nav>

          <Route path = "/" exact component = {Controls} />
          <Route path = "/vision" component = {Vision} />
          <Route path = "/settings" component = {Settings} />
        </div>
      </Router>
    );
  }
}

export default App;
