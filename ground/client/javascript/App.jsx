// Home Interface
import React, { Component } from "react";
import Navbar from "./Navbar";
import Panel from "./Panel";
import Screen from "./Screen";

class App extends Component {

  render() {
    return (
      <div className="App">

        <Navbar/>
        <Panel/>
        <Screen/>

      </div>
    );
  }
}

export default App;