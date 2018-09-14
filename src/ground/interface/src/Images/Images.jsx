import React, { Component } from "react";
import './Images.css';
import logo from '../graphics/vector_logo.svg';
import Position from './Position';
import ModalComponent from './Modal';
import { PageHeader, ProgressBar } from 'react-bootstrap';

class Images extends Component {
  constructor (props) {
    super(props);
    this.state = {
      doubleClicked: false,
      currentPhoto: null,
      allImages: [],
      photo_lat: null,
      photo_lon: null,
      startImageIndex: 0,
      endImageIndex: 100
    }
    Object.keys(this.images).map((photo) => {
      this.state.allImages.push(photo);
    })
  }

  onImageIndexChange(event) {
      const name = event.target.name;
      this.setState({[name]: event.target.value});
  }

  render() {
    return (
      <div className="Images">
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
                  <ProgressBar style={{backgroundColor: '#e9ecef'}} now={this.state.startImageIndex/this.state.allImages.length*100} key={1} />
                  <ProgressBar striped bsStyle="info" now={(this.state.endImageIndex - this.state.startImageIndex)/this.state.allImages.length*100} key={2} />
                  <ProgressBar style={{backgroundColor: '#e9ecef'}} now={(this.state.allImages.length - this.state.endImageIndex)/this.state.allImages.length*100} key={3} />
                </ProgressBar>
              </div>
              <div style={{marginLeft: '1.5%'}} className="col-1">
                <span style={{float: 'right'}} className="badge">{this.state.allImages.length}</span>
              </div>
            </div> {/*raw-page-progress*/}
            <div id="Raw-Images-List">
              {this.renderRawImages()}
              {
                this.state.doubleClicked ? <ModalComponent image={this.state.currentPhoto}/> : null
              }
            </div> {/*raw-images-list*/}
          </div> {/*raw-images*/}
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Segmented Images</h3>
            {/* todo: Howard+Ryan - insert photos from segmented files */}
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

  gotoNextPage() {
    if (parseInt(this.state.allImages.length) - parseInt(this.state.startImageIndex) > 100)
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

  // Import images from testPhotos folder
  importAll(r) {
    let images = {};
    r.keys().map((item, index) => { images[item.replace('./', '')] = r(item); });
    return images;
  }

  // TODO(comran): Re-enable this after safely checking if photos exist.
  // images = this.importAll(require.context('../../public/testPhotos', false, /\.(png|jpe?g|svg)$/));

  renderRawImages(myList) {

    //console.log("number of images: " + this.state.allImages.length);
    return this.state.allImages
      .slice(
        this.state.startImageIndex,
        Math.min(this.state.allImages.length, this.state.endImageIndex)
      )
      .map((photo) => {
        return (
          <img className="surveyPhotoOdd"
               id={photo} src={this.images[photo]}
               onDoubleClick={() => this.photoshop(photo)}
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
    let photo_name = photo.slice(0, photo.length-4);
    fetch('/testPhotos/' + photo_name + '.json')
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
