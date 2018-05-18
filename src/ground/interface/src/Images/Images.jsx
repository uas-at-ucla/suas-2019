import React, { Component } from "react";
import './Images.css';
import logo from "../graphics/vector_logo.svg";

import { PageHeader } from "react-bootstrap";

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
        {/*<DownloadMap />*/}
        <img id="CurrentImage" src={this.state.currentImage}/>
        <div id="PotentialTargets">
        <h1>Possible Targets</h1>
        {
          this.renderPotentialTargets()
        }
        </div>

        <div id="ImageOptions">
        <button className="actionBtn" id="RecentBtn" onClick={()=>this.setState({listSelected: "Recent"})}
        onDoubleClick={()=>window.alert(this.state.recentImages)}>-Recently Viewed-</button>
        <button className="actionBtn" id="SavedBtn" onClick={()=>this.setState({listSelected: "Saved"})}>-Saved-</button>
        <button className="actionBtn" id="LatestBtn" onClick={()=>this.setState({listSelected: "Latest"})}>-Latest from Drone-</button>
        <button className="actionBtn" id="SubmittedBtn" onClick={()=>this.setState({listSelected: "Submitted"})}>-Submitted to Interop-</button>
        </div>
        <div id="ImageList">
        <div id="PhotoList">
        {
          this.renderSelection()
        }
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
    switch (this.state.listSelected)
    {
      case "Recent":
        return this.displayList(this.state.recentImages);
      case "Saved":
        return this.displayList(this.state.savedImages);
      case "Latest":
        return this.displayList(this.state.latestImages);
      case "Submitted":
        return this.displayList(this.state.submittedImages);
    }
  }
  displayList(myList) { //displays the given list of images (called above)
    this.state.allImages = [];
    /*return Object.keys(this.images).map((photo) => {
      this.state.allImages.push(photo); //this line should probalby go somewhere else!!!
      this.state.latestImages.unshift(photo); //add the new photo to the front of latestImages
      if (this.contains(myList, photo))
      {

        this.state.styleMap[photo] = '0px';
        let bdWidth = this.isSelected(photo) ? "3px" : "0px";
        return (
            <img className="surveyPhoto" style={{borderWidth: bdWidth}}
              id={photo} src={this.images[photo]} onDoubleClick={() => this.selectBtn(photo)}
              onClick={()=>this.setToCurrent(photo)} />
        );
      }
    })*/
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
