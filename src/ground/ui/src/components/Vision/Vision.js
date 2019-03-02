import React, { Component } from 'react';
import './Vision.css';
import Map from './Map/Map';
import Tagging from './Tagging/Tagging';
import Pipeline from './Pipeline/Pipeline';

class Vision extends Component {
  render() {
    return (
      <div className="Vision">
        <div className = "Map">
          <Map />
        </div>
      </div>
    );
  }
}

export default Vision;