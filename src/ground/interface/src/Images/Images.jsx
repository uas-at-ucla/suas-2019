import React, { Component } from "react";
import './Images.css';
import logo from '../graphics/vector_logo.svg';
import Position from './Position';

import { PageHeader } from 'react-bootstrap';

class Images extends Component {

  constructor (props) {
    super(props);
    this.state = {
      latestImages: [],
      allImages: [],
    }
  }

  render() {
    return (
      <div className="Images">

        <div className="row">
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Raw Images</h3>
            <div id="Raw-Images-List">
              {this.renderRawImages()}
              {/* todo: Howard+Ryan - double-click image to pop-up photo editor
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

  // Import images from testPhotos folder
  importAll(r) {
    let images = {};
    r.keys().map((item, index) => { images[item.replace('./', '')] = r(item); });
    return images;
  }

  images = this.importAll(require.context('./testPhotos', false, /\.(png|jpe?g|svg)$/));

  renderRawImages(myList) {
    Object.keys(this.images).map((photo) => {
      this.state.allImages.push(photo);
    })
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
    console.log(photo);
  }


  showPosition = (photo) => {
    // todo: show position on map + highlight the border of selected photo
    console.log(photo);
  }

  submitToInterop = () => {
    // todo: submit images to interop server
    console.log("Hello!")
  }


}

export default Images;