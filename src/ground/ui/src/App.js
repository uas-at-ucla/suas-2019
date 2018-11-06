import React, { Component } from 'react';
import './App.css';

import {BrowserRouter as Router, Route, Link} from "react-router-dom";

import Controls from './components/Controls/Controls';

const Home = () => <h2>Home</h2>;
const Vision = () => <h2>Vision</h2>;
const Settings = () => <h2>Settings</h2>;

const AppRouter = () => (
  <Router>
    <div>
      <nav>
        <ul>
          <li>
            <Link to = "/"> HOME </Link>
          </li>
          <li>
            <Link to = "/controls">CONTROLS</Link>
          </li>
          <li>
            <Link to = "/vision">VISION</Link>
          </li>
          <li>
            <Link to = "/settings">SETTINGS</Link>
          </li>
        </ul>
      </nav>

      <Route path = "/" exact component = {Home} />
      <Route path = "/controls" component = {Controls} />
      <Route path = "/vision" component = {Vision} />
      <Route path = "/settings" component = {Settings} />

    </div>
  </Router>
  
);

export default AppRouter;

/*class App extends Component {
  render() {
    return (
      <div className="App">
        <Controls/>
      </div>
    );
  }
}

export default App;*/
