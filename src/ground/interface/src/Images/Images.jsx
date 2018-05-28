import React, { Component } from "react";
import './Images.css';
import logo from '../graphics/vector_logo.svg';
import Position from './Position';
import ModalComponent from './Modal';

import { PageHeader } from 'react-bootstrap';

class Images extends Component {

  constructor (props) {
    super(props);
    this.state = {
      doubleClicked: false,
      currentPhoto: null,
      allImages: [],
    }
    Object.keys(this.images).map((photo) => {
      this.state.allImages.push(photo);
    })
  }
/*
1) double click raw image- open up to modal for cropping
2) do shit with image
3) save segmented images 
*/
  render() {
    return (
      <div className="Images">
        <div className="row">
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Raw Images</h3>
            <div id="Raw-Images-List">
              {this.renderRawImages()}
              {
                this.state.doubleClicked ? <ModalComponent image={this.state.currentPhoto}/> : null
                /* todo: Howard+Ryan - double-click image to pop-up photo editor
                  make modal into a new file */}
            </div>
          </div>
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Segmented Images</h3>
            {/* todo: Howard+Ryan - insert photos from segmented files */}
          </div>
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Position of Photo Taken</h3>
            {/* todo: Ivan - Google Maps insert here
              *Ben sends me JSON data of photo file name with
                latitude and longitude position - we get to choose
                the JSON data for Comran to setup for us
              *Every photo click will display the position on the map
              *Props must be passed down here
              */}
            <Position/>
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

  // Import images from testPhotos folder
  importAll(r) {
    let images = {};
    r.keys().map((item, index) => { images[item.replace('./', '')] = r(item); });
    return images;
  }

  images = this.importAll(require.context('../../public/testPhotos', false, /\.(png|jpe?g|svg)$/));

  renderRawImages(myList) {
    
    //console.log("number of images: " + this.state.allImages.length);
    return this.state.allImages.map((photo) => {
      return (
        <img className="surveyPhotoOdd"
             id={photo} src={this.images[photo]}
             onDoubleClick={() => this.photoshop(photo)}
             onClick={()=>this.showPosition(photo)} />
      );
    })
  }

  photoshop = (photo) => {
    // todo: Open modal photoshop editor here
    console.log("toggle: " + this.state.doubleClicked);
    this.setState({
      doubleClicked: !this.state.doubleClicked,
      currentPhoto: photo
    }); 
  }


  showPosition = (photo) => {
    // todo: show position on map + highlight the border of selected photo
    this.setState( {
      currentPhoto: photo
    });
    //console.log(photo);
  }

  submitToInterop = () => {
    // todo: submit images to interop server
    console.log("Hello!")
  }


}

export default Images;