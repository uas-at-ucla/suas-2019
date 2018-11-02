import React, { Component } from 'react';
import './App.css';

import Controls from './components/Controls/Controls';

class App extends Component {
  componentDidMount() {
    document.title = 'Flight Deck';
  }

  render() {
    return (
      <div className="App">
        <Controls/>
      </div>
    );
  }
}

export default App;
