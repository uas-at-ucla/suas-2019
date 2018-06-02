import React, { Component } from "react";
import './Images.css';
import logo from '../graphics/vector_logo.svg';
import Position from './Position';
import ModalComponent from './Modal';

import { PageHeader } from 'react-bootstrap';

const photoFolder = 'testPhotos'

class Images extends Component {
  constructor (props) {
    super(props);
    this.state = {
      doubleClicked: false,
      currentPhoto: null,
      allImages: [
        {
          id: '00019',
          src: '/'+photoFolder+'/00019.JPG'
        },
        {
          id: 'flappy',
          src: '/'+photoFolder+'/flappy.JPG'
        }
      ],
      segmentedImages: [],
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
              {
                this.state.doubleClicked ? <ModalComponent image={this.state.currentPhoto}/> : null
              }
            </div>
          </div>
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Segmented Images</h3>
            {this.renderSegmentedImages()}
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

  changeDoubleClicked() {
    this.setState({
      doubleClicked: false
    });
    console.log(this.state.doubleClicked);
  }

  componentWillReceiveProps(nextProps) {
    if (nextProps.appState.newImages !== this.props.appState.newImages) { // load finished
      for (let id of nextProps.appState.newImages.raw) {
        this.state.allImages.push({
          id: id,
          src: '/'+photoFolder+'/' + id + '.JPG'
        });

      }
      for (let id of nextProps.appState.newImages.localized) {
        this.state.segmentedImages.push({
          id: id,
          src: '/'+photoFolder+'/' + id + '.JPG'
        });
      }
      this.setState({});
    }
  }

  renderRawImages() {
    //console.log("number of images: " + this.state.allImages.length);
    return this.state.allImages.map((photo, index) => {
      return (
        <img className="surveyPhotoOdd"
             key={index}
             id={photo.id} src={photo.src}
             onDoubleClick={() => this.photoshop(photo)}
             onClick={()=>this.showPosition(photo)} />
      );
    })
  }

  renderSegmentedImages() {
    //console.log("number of images: " + this.state.allImages.length);
    return this.state.segmentedImages.map((photo, index) => {
      return (
        <img className="surveyPhotoOdd"
             key={index}
             id={photo.id} src={photo.src}
             onClick={()=>this.showPosition(photo)} />
      );
    })
  }

  photoshop = (photo) => {
    console.log("toggle: " + this.state.doubleClicked);
    this.setState({
      doubleClicked: !this.state.doubleClicked,
      currentPhoto: photo
    });
  }

  showPosition(photo) {
    // todo: highlight the border of selected photo
    let photo_name = photo.id;
    fetch('/'+photoFolder+'/' + photo_name + '.json')
      .then(res => res.json())
      .then(location_data => {
        this.setState({
          photo_lat: location_data.location.lat,
          photo_lon: location_data.location.lon
        });
      });
  }

  submitToInterop = () => {
    // todo: submit images to interop server
    console.log("Hello!")
  }
}

export default Images;