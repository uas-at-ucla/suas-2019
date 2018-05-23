import React, { Component } from "react";
import './Images.css';
import logo from '../graphics/vector_logo.svg';
import Position from './Position';

import { PageHeader } from 'react-bootstrap';

class Images extends Component {

  //gets the images out of the testPhotos folder
  importAll(r) {
    let images = {};
    r.keys().map((item, index) => { images[item.replace('./', '')] = r(item); });
    return images;
  }

  images = this.importAll(require.context('./testPhotos', false, /\.(png|jpe?g|svg)$/));

  constructor (props)
  {
    super(props);
    this.state = {
      listSelected: "Latest", /*options: Recent, Saved, Latest, Submitted*/
      selectedImages: [],
      recentImages: [],
      savedImages: [],
      latestImages: [],
      submittedImages: [],
      allImages: [],
      styleMap: {},
      currentImage: ""
    }
  }


  render() {

    return (
      <div className="Images">
        <div className="row">
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Raw Images</h3>
            <div id="Raw-Images-List">
              {this.renderSelection()}
              {/* todo: Howard+Ryan - double-click image to pop-up photo editor
                  make modal into a new file */}
            </div>
          </div>
          <div className="col-md-4 col-sm-4 col-xs-4 text-center">
            <h3>Segmented Images</h3>
            {/* todo: Howard+Ryan - insert photos from segmented files folder;
                also set-up photo editor*/}
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
          <button className="actionBtn" id="SubmitBtn" onClick={this.submitToInterop}>Submit</button>
          <button className="actionBtn" id="ClearBtn" onClick={this.clearBtn}>Clear</button>
          <button className="actionBtn" id="SelectAllBtn" onClick={this.selectAllBtn}>Select All</button>
          <button className="actionBtn" id="DeselectAllBtn" onClick={this.deselectAllBtn}>Deselect All</button>
          <button className="actionBtn" id="SaveBtn" onClick={this.saveBtn}>Save Selection</button>
        </div>
      </div>

    );
  }

  contains(a, obj) { //checks if an array contains an object
    var i = a.length;
    while (i--) {
       if (a[i] === obj) {
           return true;
       }
    }
    return false;
  }

  renderSelection() { //chooses which set of images to display
    return this.displayList(this.state.latestImages);
  }

  displayList(myList) {
    this.state.allImages = [];
    //fix to iterate thru myList and print mylist[i]
    Object.keys(this.images).map((photo) => {
      this.state.allImages.push(photo);
      if (!this.contains(this.state.latestImages, photo))
        this.state.latestImages.unshift(photo);
    })
    return myList.map((photo) => {
      this.state.styleMap[photo] = '0px';
        let bdWidth = this.isSelected(photo) ? "3px" : "0px";
        if (myList.indexOf(photo) % 2 == 0)
          return (
              <img className="surveyPhotoEven" style={{borderWidth: bdWidth}}
                id={photo} src={this.images[photo]} onDoubleClick={() => this.selectBtn(photo)}
                onClick={()=>this.setToCurrent(photo)} />
          );
        else
          return (
            <img className="surveyPhotoOdd" style={{borderWidth: bdWidth}}
              id={photo} src={this.images[photo]} onDoubleClick={() => this.selectBtn(photo)}
              onClick={()=>this.setToCurrent(photo)} />
          );
    })
  }

  renderPotentialTargets() {
    return this.state.savedImages.map((photo) => {
      this.state.styleMap[photo] = '0px';
        let bdWidth = this.isSelected(photo) ? "3px" : "0px";
      return (
        <img className="surveyPhotoEven" style={{borderWidth: bdWidth}}
              id={photo} src={this.images[photo]} onDoubleClick={() => this.selectBtn(photo)}
              onClick={()=>this.setToCurrent(photo)} />
      )
    })
  }

  isSelected = (photoId) => { //checks if a photo is selected
    for (var i = 0; i < this.state.selectedImages.length; i++)
    {
      if (this.state.selectedImages[i] === photoId)
        return true;
    }
    return false;
  }

  selectBtn = (photoId) => { //selects or deselects a photo
    var selected = -1;
    for (var i = 0; i < this.state.selectedImages.length; i++) //find if the selected photo is already selected and deselect if it is
    {
      if (this.state.selectedImages[i] === photoId)
      {
        selected = i;
        this.state.selectedImages.splice(i, 1);
        //window.alert("deselected image: " + photoId );
        this.state.styleMap[photoId] = '0px';
        //return false;
        break;
      }
    }
    if (selected == -1)
    {
      this.state.selectedImages.push(photoId);
      //window.alert("selected image: " + photoId);
      this.state.styleMap[photoId] = '3px';
      //return true;
    }
  }

  setToCurrent = (photoId) => {
          //set currentImage to keys{photo}
        this.setState({currentImage: this.images[photoId]});
        //make sure recentImages only contains each photo once -> delete all other instances of photo
        for (var i = 0 ; i < this.state.recentImages.length; i++)
        {
          if (this.state.recentImages[i] === photoId)
            this.state.recentImages.splice(i, 1);
        }
        this.state.recentImages.unshift(photoId); //add to front of recent images

  }

  saveBtn = () => {
    //loop thru selected images
    //if image is not already in saved list, append to saved list
    for (var i = 0; i < this.state.selectedImages.length; i++)
    {
      if (!this.contains(this.state.savedImages, this.state.selectedImages[i]))
      {
        this.state.savedImages.push(this.state.selectedImages[i]);
      }
    }
    this.state.selectedImages = []; //clear selected images
  }

  selectAllBtn = () => {
    var selectAll = "selected images: \n";
    //select the images which are currently displayed in the ImageList
    switch (this.state.listSelected)
    {//figure out which list is currently displayed
    case "Recent":
      this.state.selectedImages = Array.from(this.state.recentImages);
      break;
    case "Saved":
      this.state.selectedImages = Array.from(this.state.savedImages);
      break;
    case "Latest":
      this.state.selectedImages = Array.from(this.state.latestImages);
      break;
    case "Submitted":
      this.state.selectedImages = Array.from(this.state.submittedImages);
      break;
    }
    for (var i = 0; i < this.state.selectedImages.length; i++)
    {
      selectAll += this.state.selectedImages[i] + '\n';
    }
    window.alert(selectAll);
  }

  deselectAllBtn = () => {
    this.state.selectedImages = [];
  }

  getTestImage = () => {
    this.props.socketEmit('test_image', {fileName: 'send_to_ground_test.jpg'});
  }

  submitToInterop = () => {
    //code to submit selected images to interop
    //change this later lol
    var submit = "Submitting the following images: \n";
    var alert = "\nThe following have already been submited: \n"
    for (var i = 0; i < this.state.selectedImages.length; i++)
    {
      if (!this.contains(this.state.submittedImages, this.state.selectedImages[i]))
      {  this.state.submittedImages.push(this.state.selectedImages[i]);
          submit += (this.state.selectedImages[i] + "\n");
      }
      else
      alert += (this.state.selectedImages[i]);
    }
    this.state.selectedImages = []; //clear selected images
    window.alert(submit + alert);
  }

  clearBtn = () => {
    if (window.confirm('Are you sure you want to clear all selected images?'))
    {
      var clear = "Clearing the following images: \n";
      for (var i = 0; i < this.state.selectedImages.length; i++)
      {
        clear += (this.state.selectedImages[i] + '\n');
      }
      window.alert(clear);
      for (var i = 0; i < this.state.allImages.length; i++) //remove all selected images from both selectedImages and allImages
      {
        for (var k = 0; k < this.state.selectedImages.length; k++)
        {
          if (this.state.allImages[i] === this.state.selectedImages[k])
          {
            this.state.allImages.splice(i, 1);
            this.state.selectedImages.splice(k, 1);
            i--;
            k--;
          }
        }
      }
      if (this.state.selectedImages.length != 0) window.alert("all selected images not cleared!");
      this.imageList = {};
    }
  }
}

export default Images;
