import React, { Component } from 'react';
import './App.css';

import Controls from './components/Controls/Controls';
import Cosmetics from './components/Cosmetics/Cosmetics';

class App extends Component {
  componentDidMount() {
    document.title = 'Flight Deck';
  }

  render() {
    return (
      <div className="App">
        <Controls/>
        <Cosmetics/>
      </div>
    );
  }
}

export default App;
