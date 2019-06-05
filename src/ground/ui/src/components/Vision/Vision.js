import React, { Component } from 'react';
import { connect } from 'react-redux';
import {
  Container,
  Row,
  Col
} from 'reactstrap';
import './Vision.css';
import Map from './Map/Map';
import Tagging from './Tagging/Tagging';
import Pipeline from './Pipeline/Pipeline';
// import ImageCrop from './ImageCrop/ImageCrop';

const electronRequire = window.require; // could be window.require('electron').remote.require
const fs = electronRequire ? electronRequire('fs') : null;
console.log("Test filesystem API", electronRequire ? fs.readdirSync("./") : "Electron not available"); // list all files in directory

const mapStateToProps = state => { return { vision: state.vision }; };
const mapDispatchToProps = state => { };

class Vision extends Component {
  render() {
    return (
      <div className="Vision">
        <div className="map-overlay">
          <Tagging />
        </div>
        {/* <div className="image-crop">
          <ImageCrop />
        </div> */}
        <Map />
      </div>
    );
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(Vision);
