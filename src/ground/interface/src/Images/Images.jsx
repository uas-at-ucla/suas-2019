import React, { Component } from "react";
import './Images.css';
import logo from '../graphics/vector_logo.svg';
import Position from './Position';
import ModalComponent from './Modal';
import Classify from './Classify'

import { PageHeader } from 'react-bootstrap';

class Images extends Component {
  constructor (props) {
    super(props);
    this.state = {
      currentPhoto: null,
      currentCroppedPhoto: null,
      photo_lat: null,
      photo_lon: null
    }
  }

  render() {
    return (
      <div className="Images">
        <div className="row">
          <div className="col-md-4 col-sm-4 col-xs-4 text-center"
               id="raw-images">
            <h3>Raw Images</h3>
            <div id="Raw-Images-List">
              {this.renderRawImages()}
              {this.state.currentPhoto ? 
                <ModalComponent image={this.state.currentPhoto} 
                                doubleClick={this.state.doubleClickRaw}
                                socketEmit={this.props.socketEmit}/> : null
              }
            </div>
          </div>
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Segmented Images</h3>
            <div id="Segmented-Images-List">
              {this.renderSegmentedImages()}
              {this.state.currentCroppedPhoto ? 
                <Classify image={this.state.currentCroppedPhoto} 
                          doubleClick={this.state.doubleClickCropped}
                          socketEmit={this.props.socketEmit}
                          object={this.state.submittedObject}/> : null
              }
            </div>
          </div>
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Position of Photo Taken</h3>
            <Position lon={this.state.photo_lon}
                      lat={this.state.photo_lat}/>
          </div>
        </div>

        <div id="actionBar">
          {/*This must select all segmented images  */}
          <button className="actionBtn" id="SelectAllBtn" onClick={this.selectAllBtn}>Select All</button>
          <button className="actionBtn" id="SubmitBtn" onClick={this.submitToInterop}>Submit</button>
        </div>
      </div>

    );
  }

  renderRawImages() {
    //console.log("number of images: " + this.props.appState.rawImages.length);
    return this.props.appState.rawImages.map((photo, index) => {
      return (
        <img className="surveyPhotoOdd"
             key={index}
             id={photo.id} src={photo.src}
             onDoubleClick={() => this.photoshop(photo)}
             onClick={()=>this.showPosition(photo)}
             style={this.rawImageStyle(photo.id)} />
      );
    })
  }

  renderSegmentedImages() {
    return this.props.appState.segmentedImages.map((photo, index) => {
      return (
        <img className="surveyPhotoOdd"
             key={index}
             id={photo.id} src={photo.src}
             onDoubleClick={() => this.classify(photo)}
             onClick={()=>this.showPosition(photo)}
             style={this.segmentedImageStyle(photo.id)} />
      );
    })
  }

  segmentedImageStyle(id) {
    if (this.props.appState.manualClassifiedImages.find(el => el.id === id)) {
      return {borderBottom: "4px solid rgba(0, 255, 0, 2)"};
    } else if (this.props.appState.manualCroppedImageParents.find(el => el === this.props.appState.croppedImages[id])) {
      return {borderBottom: "4px solid rgba(255, 0, 0, 2)"};
    } else {
      return {};
    }
  }

  rawImageStyle(id) {
    if (this.props.appState.manualClassifiedImages.find(el => this.props.appState.croppedImages[el.id] === id)) {
      return {borderBottom: "4px solid rgba(0, 255, 0, 2)"};
    } else if (this.props.appState.manualCroppedImageParents.find(el => el === id)) {
      return {borderBottom: "4px solid rgba(255, 0, 0, 2)"};
    } else {
      return {};
    }
  }

  photoshop = (photo) => {
    this.setState({
      doubleClickRaw: {},
      currentPhoto: photo
    });
  }

  classify = (photo) => {
    let object = null;
    let classified = this.props.appState.manualClassifiedImages.find(el => el.id === photo.id);
    if (classified) {
      object = classified.object;
    }
    this.setState({
      doubleClickCropped: {},
      currentCroppedPhoto: photo,
      submittedObject: object
    });
  }

  showPosition(photo) {
    // todo: highlight the border of selected photo
    let photo_name = photo.id;
    fetch('/'+this.props.appState.photoFolder+'/' + photo_name + '.json')
      .then(res => res.json())
      .then(location_data => {
        this.setState({
          photo_lat: location_data.location.lat,
          photo_lon: location_data.location.lon
        });
      });
  }
}

export default Images;