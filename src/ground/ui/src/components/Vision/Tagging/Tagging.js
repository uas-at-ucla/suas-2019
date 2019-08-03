<<<<<<< HEAD
import React, { Component } from "react";
import Select from "react-select";
import "./Tagging.css";
import ReactCrop from "react-easy-crop";
import { Button, Modal } from "reactstrap";
import imageClipper from 'image-clipper';
import { connect } from 'react-redux';

// import testImage from "./testImages/pexels-photo-236047.jpeg";

const ListItem = ({ value, onClick }) => <li onClick={onClick}>{value}</li>;

const List = ({ items, onItemClick }) =>
  items != null ? (
    <ul>
      {items.map((item, i) => (
        <ListItem key={i} value={item} onClick={onItemClick} />
      ))}
    </ul>
  ) : null;

const path = window.require('path');
const electron = window.require("electron");
const fs = electron.remote.require("fs");
// const imagePath = "../ui/src/components/Vision/Tagging/testImages/";
let images = [];//fs.readdirSync(imagePath);
=======
import React, { Component } from 'react';
import Select from 'react-select';
import './Tagging.css';
import ReactCrop from 'react-easy-crop';
import { Button, Modal } from 'reactstrap';
import * as path from 'path';

import testImage from './testImages/18mmZoom.JPG';

const ListItem = ({ value, onClick }) => (
  <li onClick={onClick}>{value}</li>
);

const List = ({ items, onItemClick }) => (
  (items != null) ? (
    <ul>
      {
        items.map((item, i) => <ListItem key={i} value={item} onClick={onItemClick} />)
      }
    </ul>) : null
);

const electron = window.require('electron');
const fs = electron.remote.require('fs');
const imagePath = "../ui/src/components/Vision/Tagging/testImages/";
let images = fs.readdirSync(imagePath);
>>>>>>> vision_frontend

class Tagging extends Component {
  constructor(props) {
    super(props);
    this.state = {
      formValues: {},
      annValues: {
<<<<<<< HEAD
        shape: "",
        shapeCol: "",
        letter: "",
        letterCol: "",
        orient: ""
      },
      selectedImage: null /*testImage*/,
      // croppedImage: null,
=======
        shape: '',
        shapeCol: '',
        letter: '',
        letterCol: '',
        orient: '',
      },
      selectedImage: testImage,
      croppedImage: null,
      displayImage: null,
>>>>>>> vision_frontend
      crop: {
        x: 130,
        y: 50,
        width: 0,
<<<<<<< HEAD
        height: 0
      },
      zoom: 1,
      aspect: 1 / 1,
      displayCropper: false,
      croppedAreaPixels: null, //result from image crop
      imageList: images
=======
        height: 0,
      },
      zoom: 1,
      aspect: 4 / 3,
      displayCropper: false,
      croppedAreaPixels: null, //result from image crop
      imageList: images,
>>>>>>> vision_frontend
    };
    this.handleSubmit = this.handleSubmit.bind(this);
    this.handleChange = this.handleChange.bind(this);
  }

  handleChange(event) {
    event.preventDefault();
    let name = event.target.name;
    let value = event.target.value;
    let formValues = this.state.formValues;
    formValues[name] = value;
    // console.log("click value: ", value);
<<<<<<< HEAD
    this.setState(state => {
      return {
        formValues //, curValues
      };
    });
  }

  handleSelectChange = (name, selectedOption) => {
    let value = selectedOption.value;
    let formValues = this.state.formValues;
    formValues[name] = value;
    console.log(name + ` selected:`, value);
    this.setState(state => {
=======
    this.setState((state) => {
      return {
        formValues  //, curValues
      }
    });
  }

  handleSelectChange = (selectedOption) => {
    let name = selectedOption.label;
    let value = selectedOption.value;
    let formValues = this.state.formValues;
    formValues[name] = value;
    console.log(`Option selected:`, value);
    this.setState((state) => {
>>>>>>> vision_frontend
      return {
        formValues //, curValues
      };
    });
<<<<<<< HEAD
  };
  handleRestore = () => {
    this.setState({
      // croppedImage: null,
      crop: {
        x: 0,
        y: 0,
        width: 0,
        height: 0
      },
      zoom: 1
    });
  };

  handleCropComplete = () => {
    let crop = this.state.croppedAreaPixels;
    console.log("cropping...")
    // let croppedImg = this.getCroppedImg(this.refImageCrop, crop);
    // console.log("width", croppedImg.width);
    let self = this;
    imageClipper(this.state.selectedImage, function() {
      this.crop(crop.x, crop.y, crop.width, crop.height)
      .toDataURL((dataURL) => {
        let base64Data = dataURL.split(';base64,').pop();
        let extension = path.extname(self.state.imagePath).split(".").pop();
        let croppedImagePath = self.state.imagePath.split('.')[0] + "_cropped." + extension;
        fs.writeFile(croppedImagePath, base64Data, {encoding: 'base64'}, (err) => {
          if (err) {
            console.log(err);
          } else {
            console.log("Saved cropped image", croppedImagePath);
            self.setState({croppedImagePath: croppedImagePath});
          }
        });
      });
    });
    // this.setState({ displayImage: croppedImg, croppedImage: croppedImg })
  }

  // onImageLoaded = (image) => {
  //   this.setState({
  //     crop: makeAspectCrop({
  //       x: 0,
  //       y: 0,
  //       // aspect: 10 / 4,
  //       // width: 50,
  //     }, image.naturalWidth / image.naturalHeight),
  //     image,
  //   });
  // }

  handleOpen = () => {
    // console.log("image cropper opened");
    let displayCropper = !this.state.displayCropper;
    // let displayImage = this.state.selectedImage;
    this.setState({ displayCropper: displayCropper });
    // this.setState({ displayImage: displayImage });
  };
=======
  }
  handleRestore = () => {
    this.setState({
      displayImage: this.state.selectedImage,
      croppedImage: null,
      zoom: 1,
    });
  }

  handleCropComplete() {
    // let crop = this.state.crop;
    // let croppedImg = this.getCroppedImg(this.refImageCrop, crop);
    // console.log("width", croppedImg.width);
    console.log("Cropped");
    // this.setState({ displayImage: croppedImg, croppedImage: croppedImg })
  }

  // onImageLoaded = (image) => {
  //   this.setState({
  //     crop: makeAspectCrop({
  //       x: 0,
  //       y: 0,
  //       // aspect: 10 / 4,
  //       // width: 50,
  //     }, image.naturalWidth / image.naturalHeight),
  //     image,
  //   });
  // }

  handleOpen = () => {
    // console.log("image cropper opened");
    let displayCropper = !this.state.displayCropper;
    let displayImage = this.state.selectedImage;
    this.setState({ displayCropper: displayCropper });
    this.setState({ displayImage: displayImage });

  }
>>>>>>> vision_frontend

  handleSubmit(event) {
    event.preventDefault();
    let formValues = this.state.formValues;
    let annValues = this.state.annValues;

    for (var key in formValues) {
<<<<<<< HEAD
      if (formValues[key] != "") {
=======
      if (formValues[key] != '') {
>>>>>>> vision_frontend
        annValues[key] = formValues[key];
        formValues[key] = "";
      }
    }

<<<<<<< HEAD
    this.setState(state => {
      return {
        annValues,
        formValues
      };
    });
  }

  handleInteropSubmit = () => {
    this.props.dispatch({
      type: 'TRANSMIT',
      payload: {
        msg: 'UPLOAD_IMAGE',
        data: {...this.state.annValues, imageFile: this.state.croppedImagePath} // TODO get selected latitude & longitude
      }
    });
  }

  onCropChange = crop => {
    this.setState({ crop });
  };

  onCropComplete = (croppedArea, croppedAreaPixels) => {
    this.setState({ croppedAreaPixels: croppedAreaPixels });
    console.log("croppedAreaPixels", croppedAreaPixels);
    // this.setState({crop: croppedArea});
  };

  onZoomChange = zoom => {
    this.setState({ zoom });
  };

  handleListClick = e => {
    let imageName = e.target.innerHTML;
    let imageP = /*imagePath + */imageName;
    fs.readFile(imageP, (err, data) => {
      if (err) {
        console.error(err);
        return;
      }
      let extensionName = path.extname(imageP).split(".").pop();
      let base64Image = new Buffer(data, "binary").toString("base64");
      let imgSrcString = `data:image/${extensionName};base64,${base64Image}`;
      this.setState({
        selectedImage: imgSrcString,
        croppedImagePath: null,
        // displayImage: imgSrcString,
        imagePath: path.resolve(imageP),
        imageName: imageName,
        annValues: {...this.state.annValues}
      });
    });
  };

=======
    this.setState((state) => {
      return {
        annValues, formValues
      }
    });
  }

  onCropChange = crop => {
    this.setState({ crop })
  }

  onCropComplete = (croppedArea, croppedAreaPixels) => {
    this.setState({ croppedAreaPixels: croppedAreaPixels });
    console.log("croppedAreaPixels", croppedAreaPixels);
    // this.setState({crop: croppedArea});
  }

  onZoomChange = zoom => {
    this.setState({ zoom })
  }

  handleListClick = (e) => {
    let imageP = imagePath + e.target.innerHTML;
    fs.readFile(imageP, (err, data) => {
      if (err) {
        console.error(err)
        return
      }
      let extensionName = path.extname(imageP);
      let base64Image = new Buffer(data, 'binary').toString('base64');
      let imgSrcString = `data:image/${extensionName.split('.').pop()};base64,${base64Image}`;
      this.setState({ selectedImage: imgSrcString });
      this.setState({ displayImage: imgSrcString });
    })
  }

>>>>>>> vision_frontend
  render() {
    return (
      <div className="Annotations">
        <h1>Annotations</h1>
        <form id="Annotate" onSubmit={this.handleSubmit}>
          <div className="Shape">
<<<<<<< HEAD
            <label className="dualField">
              {" "}
              <span className="fieldVal">Shape: </span>{" "}
              <span className="curVal">{this.state.annValues["shape"]}</span>
              <br />
              <Select
                name="shape"
                isSearchable="true"
                placeholder="Shape"
                onChange={(option) => this.handleSelectChange("shape", option)}
                options={shapeOptions}
                styles={selectStyles}
              />
            </label>
            <label className="dualField">
              {" "}
              <span className="fieldVal">Color: </span>{" "}
              <span className="curVal">{this.state.annValues["shapeCol"]}</span>
              <br />
              <Select
                name="shapeCol"
                isSearchable="true"
                placeholder="Shape Color"
                onChange={(option) => this.handleSelectChange("shapeCol", option)}
                options={colorOptions}
                styles={selectStyles}
              />
=======
            <label className="dualField"> <span className="fieldVal">Shape: </span> <span className="curVal">{this.state.annValues["shape"]}</span>
              <br></br><Select name="shape" isSearchable='true' placeholder="Shape" value={this.state.formValues["shape"]}
                onChange={this.handleSelectChange} options={shapeOptions} styles={selectStyles} />
            </label>
            <label className="dualField"> <span className="fieldVal">Color: </span> <span className="curVal">{this.state.annValues["shapeCol"]}</span>
              <br></br><Select name="shapeCol" isSearchable='true' placeholder="Shape Color" value={this.state.formValues["shapeCol"]}
                onChange={this.handleSelectChange} options={colorOptions} styles={selectStyles} />
>>>>>>> vision_frontend
            </label>
          </div>

          <div className="Letter">
<<<<<<< HEAD
            <label className="dualField">
              {" "}
              <span className="fieldVal">Letter: </span>{" "}
              <span className="curVal">{this.state.annValues["letter"]}</span>
              <br />
              <input
                type="text"
                name="letter"
                placeholder="Letter"
                value={this.state.formValues["letter"]}
                onChange={this.handleChange}
              />
            </label>
            <label className="dualField">
              {" "}
              <span className="fieldVal">Color: </span>{" "}
              <span className="curVal">
                {this.state.annValues["letterCol"]}
              </span>
              <br />
              <Select
                name="letterCol"
                isSearchable="true"
                placeholder="Letter Color"
                onChange={(option) => this.handleSelectChange("letterCol", option)}
                options={colorOptions}
                styles={selectStyles}
              />
=======
            <label className="dualField"> <span className="fieldVal">Letter: </span> <span className="curVal">{this.state.annValues["letter"]}</span>
              <br></br><input type="text" name="letter" placeholder="Letter" value={this.state.formValues["letter"]} onChange={this.handleChange} />
            </label>
            <label className="dualField"> <span className="fieldVal">Color: </span> <span className="curVal">{this.state.annValues["letterCol"]}</span>
              <br></br><Select name="letterCol" isSearchable='true' placeholder="Letter Color" value={this.state.formValues["letterCol"]}
                onChange={this.handleSelectChange} options={colorOptions} styles={selectStyles} />
>>>>>>> vision_frontend
            </label>
          </div>

          <div className="Orient">
<<<<<<< HEAD
            <label className="singField">
              {" "}
              <span className="fieldVal">Orientation (N, NE, etc.): </span>{" "}
              <span className="curVal">{this.state.annValues["orient"]}</span>
              <br />
              <input
                type="text"
                name="orient"
                placeholder="Orientation"
                value={this.state.formValues["orient"]}
                onChange={this.handleChange}
              />
=======
            <label className="singField"> <span className="fieldVal">Orientation: </span> <span className="curVal">{this.state.annValues["orient"]}</span>
              <br></br><input type="text" name="orient" placeholder="Orientation" value={this.state.formValues["orient"]} onChange={this.handleChange} />
>>>>>>> vision_frontend
            </label>
          </div>

          <div className="saveButton">
<<<<<<< HEAD
            <input
              options={colorOptions}
              styles={selectStyles}
              me="btn btn-primary"
              type="submit"
              value="Save"
            />
            {this.state.displayCropper && this.state.selectedImage ? (
              <div>
                <Button
                  color="primary"
                  onClick={this.handleRestore}
                >
                  Restore
                </Button>
                <Button
                  color="primary"
                  keyboardFocused={true}
                  onClick={this.handleCropComplete}
                >
                  Crop
                </Button>
                <Button
                  color="success"
                  keyboardFocused={true}
                  onClick={this.handleInteropSubmit}
                  disabled={!this.state.croppedImagePath}
                >
                  {this.state.croppedImagePath ? "Submit to Interop" : "Waiting for Crop"}
                </Button>
              </div>
            ) : null}
=======
            <input classNaonChange={this.handleSelectChange} options={colorOptions} styles={selectStyles} me="btn btn-primary" type="submit" value="Save" />
            {this.state.displayCropper ?
              <div>
                <Button variant="primary" primary={true} onClick={this.handleRestore}>Restore</Button>
                <Button variant="primary" primary={true} keyboardFocused={true} onClick={this.handleCropComplete} >Crop</Button>
              </div> : null}

>>>>>>> vision_frontend
          </div>
        </form>

        <div className="imageList">
          <div className="scroller">
<<<<<<< HEAD
            <div>Selected: {this.state.imageName}</div>
            <List
              items={this.state.imageList}
              onItemClick={this.handleListClick}
            />
=======
            <List items={this.state.imageList} onItemClick={this.handleListClick} />
>>>>>>> vision_frontend
          </div>
        </div>

        <div className="imageButton">
<<<<<<< HEAD
          <Button color="primary" onClick={this.handleOpen}>
            {this.state.selectedImage ? "Toggle image cropper" : "Select an image first"}
          </Button>
          {this.state.displayCropper && this.state.selectedImage ? (
            <div className="imageCrop">
              <ReactCrop
                image={this.state.selectedImage}
=======
          <Button variant="primary" onClick={this.handleOpen}>Toggle image cropper</Button>
          {this.state.displayCropper ?
            <div className="imageCrop">
              <ReactCrop
                image={this.state.displayImage}
>>>>>>> vision_frontend
                crop={this.state.crop}
                zoom={this.state.zoom}
                aspect={this.state.aspect}
                onCropChange={this.onCropChange}
                onCropComplete={this.onCropComplete}
                onZoomChange={this.onZoomChange}
                maxZoom={1000}
                zoomSpeed={2}
              />
<<<<<<< HEAD
            </div>
          ) : null}
          <img
            src={this.state.selectedImage}
            style={{ display: "none" }}
            ref={img => {
              this.refImageCrop = img;
            }}
            alt=""
          />
          {/* <img src={this.state.croppedImage} alt="" /> */}
        </div>
      </div>
    );
  }
=======
            </div> : null}
          <img src={this.state.displayImage} style={{ display: "none" }} ref={(img) => { this.refImageCrop = img }} alt="" />
          {/* <img src={this.state.croppedImage} alt="" /> */}
        </div>

      </div>
    )
  }
}

const shapeOptions = [
  { label: "Circle", value: "Circle" },
  { label: "Cross", value: "Cross" },
  { label: "Heptagon", value: "Heptagon" },
  { label: "Hexagon", value: "Hexagon" },
  { label: "Octagon", value: "Octagon" },
  { label: "Pentagon", value: "Pentagon" },
  { label: "QuarterCircle", value: "QuarterCircle" },
  { label: "Rectangle", value: "Rectangle" },
  { label: "SemiCircle", value: "SemiCircle" },
  { label: "Square", value: "Square" },
  { label: "Star", value: "Star" },
  { label: "Trapezoid", value: "Trapezoid" },
  { label: "Triangle", value: "Triangle" },
];

const colorOptions = [
  { label: "black", value: "black" },
  { label: "blue", value: "blue" },
  { label: "brown", value: "brown" },
  { label: "gray", value: "gray" },
  { label: "green", value: "green" },
  { label: "orange", value: "orange" },
  { label: "purple", value: "purple" },
  { label: "red", value: "red" },
  { label: "yellow", value: "yellow" },
];

const selectStyles = {
  control: (styles) => ({ ...styles, backgroundColor: 'white', height: 20 }),
  option: (styles, { data, isDisabled, isFocused, isSelected }) => {
    // const color = chroma(data.color);
    return {
      ...styles,
      //color of options' background: gray
      backgroundColor: isDisabled ? 'red' : '#7e7e7e',
      color: '#FFF',
      cursor: isDisabled ? 'not-allowed' : 'default',
    };
  },
  //menu list height
  menuList: (styles) => ({ ...styles, height: 200, }),
>>>>>>> vision_frontend
}

const shapeOptions = [
  { label: "Circle", value: "CIRCLE" },
  { label: "SemiCircle", value: "SEMICIRCLE" },
  { label: "QuarterCircle", value: "QUARTER_CIRCLE" },
  { label: "Triangle", value: "TRIANGLE" },
  { label: "Square", value: "SQUARE" },
  { label: "Rectangle", value: "RECTANGLE" },
  { label: "Trapezoid", value: "TRAPEZOID" },
  { label: "Pentagon", value: "PENTAGON" },
  { label: "Hexagon", value: "HEXAGON" },
  { label: "Heptagon", value: "HEPTAGON" },
  { label: "Octagon", value: "OCTAGON" },
  { label: "Star", value: "STAR" },
  { label: "Cross", value: "CROSS" }
];

const colorOptions = [
  { label: "white", value: "WHITE" },
  { label: "black", value: "BLACK" },
  { label: "gray", value: "GRAY" },
  { label: "red", value: "RED" },
  { label: "blue", value: "BLUE" },
  { label: "green", value: "GREEN" },
  { label: "yellow", value: "YELLOW" },
  { label: "purple", value: "PURPLE" },
  { label: "brown", value: "BROWN" },
  { label: "orange", value: "ORANGE" }
];

const selectStyles = {
  control: styles => ({ ...styles, backgroundColor: "white", height: 20 }),
  option: (styles, { data, isDisabled, isFocused, isSelected }) => {
    // const color = chroma(data.color);
    return {
      ...styles,
      //color of options' background: gray
      backgroundColor: isDisabled ? "red" : "#7e7e7e",
      color: "#FFF",
      cursor: isDisabled ? "not-allowed" : "default"
    };
  },
  //menu list height
  menuList: styles => ({ ...styles, height: 200 })
};

export default connect()(Tagging);
