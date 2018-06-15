import React, { Component } from "react";
import './Images.css';
import logo from '../graphics/vector_logo.svg';
import Position from './Position';
import ModalComponent from './Modal';
import { PageHeader, ProgressBar } from 'react-bootstrap';
import Classify from './Classify'

const colorKey = {
  manualCrop:     "rgba(255, 0,   0,   1)",
  manualClassify: "rgba(0,   255, 0,   1)",
  autoCrop:       "rgba(255, 127, 0,   1)",
  autoClassify:   "rgba(0,   255, 255, 1)"
}

class Images extends Component {
  constructor (props) {
    super(props);
    this.state = {
      currentPhoto: null,
      currentCroppedPhoto: null,
      photo_lat: null,
      photo_lon: null,
      startImageIndex: 0,
      endImageIndex: 100
    }
  }

  onImageIndexChange(event) {
      const name = event.target.name;
      this.setState({[name]: event.target.value});
  }

  onImageIndexChange(event) {
      const name = event.target.name;
      this.setState({[name]: event.target.value});
  }

  render() {
    return (
      <div className="Images">
        <div class="image_color_key">
          <span style={{backgroundColor: colorKey.manualCrop}}>Manual-Cropped</span>
          <span style={{backgroundColor: colorKey.manualClassify}}>Manual-Classified</span>
          <span style={{backgroundColor: colorKey.autoCrop}}>Auto-Cropped</span>
          <span style={{backgroundColor: colorKey.autoClassify}}>Auto-Classified</span>
        </div>
        <div className="row">
          <div className="col-md-4 col-sm-4 col-xs-4 text-center"
               id="raw-images">
            <h3>Raw Images</h3>
            <div className="row" id="raw-page-controls">
              <div className="input-group input-group-sm col">
                <div className="input-group-prepend">
                  <button
                    type="button"
                    className="btn btn-outline-secondary"
                    onClick={this.gotoPrevPage.bind(this)}
                  >
                    -100
                  </button>
                </div>
                <input
                  name="startImageIndex"
                  className="form-control"
                  type="number"
                  onChange={this.onImageIndexChange.bind(this)}
                  value={this.state.startImageIndex}
                />
                <div className="input-group-prepend input-group-append">
                  <span className="input-group-text">to</span>
                </div>
                <input
                  name="endImageIndex"
                  className="form-control"
                  type="number"
                  onChange={this.onImageIndexChange.bind(this)}
                  value={this.state.endImageIndex}
                />
                <div className="input-group-append">
                  <button
                    type="button"
                    className="btn btn-outline-secondary"
                    onClick={this.gotoNextPage.bind(this)}
                  >
                    +100
                  </button>
                </div>
              </div>
            </div> {/*raw-page-controls*/}
            <div style={{marginTop: '3%'}} className="row" id="raw-page-progress">
              <div className="col-1">
                <span style={{float: 'left'}} className="badge">0</span>
              </div>
              <div className="col">
                <ProgressBar>
                  <ProgressBar style={{backgroundColor: '#e9ecef'}} now={this.state.startImageIndex/this.props.appState.rawImages.length*100} key={1} />
                  <ProgressBar striped bsStyle="info" now={(this.state.endImageIndex - this.state.startImageIndex)/this.props.appState.rawImages.length*100} key={2} />
                  <ProgressBar style={{backgroundColor: '#e9ecef'}} now={(this.props.appState.rawImages.length - this.state.endImageIndex)/this.props.appState.rawImages.length*100} key={3} />
                </ProgressBar>
              </div>
              <div style={{marginLeft: '1.5%'}} className="col-1">
                <span style={{float: 'right'}} className="badge">{this.props.appState.rawImages.length}</span>
              </div>
            </div> {/*raw-page-progress*/}
            <div id="Raw-Images-List">
              {this.renderRawImages()}
              {this.state.currentPhoto ?
                <ModalComponent image={this.state.currentPhoto}
                                doubleClick={this.state.doubleClickRaw}
                                socketEmit={this.props.socketEmit}/> : null
              }
            </div> {/*raw-images-list*/}
          </div> {/*raw-images*/}
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Segmented Images</h3>
            <div id="Segmented-Images-List">
              {this.renderSegmentedImages()}
              {this.state.currentCroppedPhoto ?
                <Classify image={this.state.currentCroppedPhoto}
                          doubleClick={this.state.doubleClickCropped}
                          socketEmit={this.props.socketEmit}
                          object={this.state.classifiedObject}
                          submitted={this.state.objectSubmitted}
                          appState={this.props.appState}/> : null
              }
            </div>
          </div>
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Position of Photo Taken</h3>
            <Position lon={this.state.photo_lon}
                      lat={this.state.photo_lat}/>
          </div>
        </div>

        {/*<div id="actionBar">
          {//This must select all segmented images
          }
          <button className="actionBtn" id="SelectAllBtn" onClick={this.selectAllBtn}>Select All</button>
          <button className="actionBtn" id="SubmitBtn" onClick={this.submitToInterop}>Submit</button>
        </div>*/}
      </div>
    );
  }

  gotoNextPage() {
    if (parseInt(this.props.appState.rawImages.length) - parseInt(this.state.startImageIndex) > 100)
      this.setState({
        startImageIndex: parseInt(this.state.startImageIndex) + 100,
        endImageIndex: parseInt(this.state.endImageIndex) + 100
      });
  }

  gotoPrevPage() {
    if (parseInt(this.state.startImageIndex) != 0)
      this.setState({
        startImageIndex: parseInt(this.state.startImageIndex) - 100,
        endImageIndex: parseInt(this.state.endImageIndex) - 100
      });
  }

  renderRawImages() {
    //console.log("number of images: " + this.props.appState.rawImages.length);
    return this.props.appState.rawImages
      .slice(
        this.state.startImageIndex,
        Math.min(this.props.appState.rawImages.length, this.state.endImageIndex)
      ).map((photo, index) => {
      return (
        <img className="individual-raw-photo"
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
    let style = {};

    if (this.props.appState.manualCroppedImageParents.find(el => el === this.props.appState.croppedImages[id])) {
      style.borderBottom = "4px solid " + colorKey.manualCrop;
    } else {
      style.borderTop = "4px solid " + colorKey.autoCrop;
    }

    if (this.props.appState.manualClassifiedImages.find(el => el.id === id)) {
      style.borderBottom = "4px solid " + colorKey.manualClassify;
    }

    if (this.props.appState.autoClassifiedImages.find(el => el === id)) {
      style.borderTop = "4px solid " + colorKey.autoClassify;
    }

    return style;
  }

  rawImageStyle(id) {
    let style = {};

    if (this.props.appState.manualCroppedImageParents.find(el => el === id)) {
      style.borderBottom = "4px solid " + colorKey.manualCrop;
    } else if (Object.keys(this.props.appState.croppedImages).find(el => this.props.appState.croppedImages[el] === id)) {
      style.borderTop = "4px solid " + colorKey.autoCrop;
    }

    if (this.props.appState.manualClassifiedImages.find(el => this.props.appState.croppedImages[el.id] === id)) {
      style.borderBottom = "4px solid " + colorKey.manualClassify;
    }

    if (this.props.appState.autoClassifiedImages.find(el => this.props.appState.croppedImages[el] === id)) {
      style.borderTop = "4px solid " + colorKey.autoClassify;
    }

    return style;
  }

  photoshop = (photo) => {
    this.setState({
      doubleClickRaw: {},
      currentPhoto: photo
    });
  }

  classify = (photo) => {
    let object = null;
    let submitted = false;
    let classified = this.props.appState.manualClassifiedImages.find(el => el.id === photo.id);
    if (classified) {
      object = classified.object;
      submitted = true;
    } else {
      fetch('/'+this.props.appState.photoFolder+'/' + photo.id + '.json')
        .then(res => res.json())
        .catch(error => console.log("No JSON file exists!"))
        .then(response => {
          let auto_object = {};
          auto_object.latitude = response.location.lat || 0;
          auto_object.longitude = response.location.lng || 0;
          auto_object.shape = response.shape || "";
          auto_object.background_color = response.target_color || "";
          auto_object.alphanumeric = response.letter || "";
          auto_object.alphanumeric_color = response.letter_color || "";
          auto_object.orientation = response.orientation || "";
          this.setState({
            classifiedObject: auto_object,
          });
        })
        .catch(error => console.log("Fetch request failed."))
    }
    this.setState({
      doubleClickCropped: {},
      currentCroppedPhoto: photo,
      classifiedObject: object,
      objectSubmitted: submitted
    });
  }

  showPosition(photo) {
    let photo_name = photo.id;
    fetch('/'+this.props.appState.photoFolder+'/' + photo_name + '.json')
      .then(res => res.json())
      .catch(error => console.log("No JSON file exists!"))
      .then(response => {
        this.setState({
          photo_lat: response.lat || response.location.lat,
          photo_lon: response.lng || response.location.lng
        });
      })
      .catch(error => console.log("Fetch request failed."))
  }
}

export default Images;
