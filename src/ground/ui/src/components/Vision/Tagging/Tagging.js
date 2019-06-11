import React, { Component } from "react";
import Select from "react-select";
import "./Tagging.css";
import ReactCrop from "react-easy-crop";
import { Button, Modal } from "reactstrap";
import imageClipper from 'image-clipper';
import { connect } from 'react-redux';

import testImage from "./testImages/pexels-photo-236047.jpeg";

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
const imagePath = "../ui/src/components/Vision/Tagging/testImages/";
let images = fs.readdirSync(imagePath);

class Tagging extends Component {
  constructor(props) {
    super(props);
    this.state = {
      formValues: {},
      annValues: {
        shape: "",
        shapeCol: "",
        letter: "",
        letterCol: "",
        orient: ""
      },
      selectedImage: null /*testImage*/,
      // croppedImage: null,
      crop: {
        x: 130,
        y: 50,
        width: 0,
        height: 0
      },
      zoom: 1,
      aspect: 1 / 1,
      displayCropper: false,
      croppedAreaPixels: null, //result from image crop
      imageList: images
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
      return {
        formValues //, curValues
      };
    });
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

  handleSubmit(event) {
    event.preventDefault();
    let formValues = this.state.formValues;
    let annValues = this.state.annValues;

    for (var key in formValues) {
      if (formValues[key] != "") {
        annValues[key] = formValues[key];
        formValues[key] = "";
      }
    }

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
    let imageP = imagePath + imageName;
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

  render() {
    return (
      <div className="Annotations">
        <h1>Annotations</h1>
        <form id="Annotate" onSubmit={this.handleSubmit}>
          <div className="Shape">
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
            </label>
          </div>

          <div className="Letter">
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
            </label>
          </div>

          <div className="Orient">
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
            </label>
          </div>

          <div className="saveButton">
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
          </div>
        </form>

        <div className="imageList">
          <div className="scroller">
            <div>Selected: {this.state.imageName}</div>
            <List
              items={this.state.imageList}
              onItemClick={this.handleListClick}
            />
          </div>
        </div>

        <div className="imageButton">
          <Button color="primary" onClick={this.handleOpen}>
            {this.state.selectedImage ? "Toggle image cropper" : "Select an image first"}
          </Button>
          {this.state.displayCropper && this.state.selectedImage ? (
            <div className="imageCrop">
              <ReactCrop
                image={this.state.selectedImage}
                crop={this.state.crop}
                zoom={this.state.zoom}
                aspect={this.state.aspect}
                onCropChange={this.onCropChange}
                onCropComplete={this.onCropComplete}
                onZoomChange={this.onZoomChange}
                maxZoom={1000}
                zoomSpeed={2}
              />
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
