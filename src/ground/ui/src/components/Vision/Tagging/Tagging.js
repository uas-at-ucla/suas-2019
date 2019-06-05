import React, { Component } from 'react';
import Select from 'react-select';
import './Tagging.css';
import ReactCrop from 'react-easy-crop';
import { Button, Modal } from 'reactstrap';
import * as path from 'path';

import testImage from './testImages/pexels-photo-236047.jpeg';

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

class Tagging extends Component {
  constructor(props) {
    super(props)
    this.state = {
      formValues: {},
      annValues: {
        shape: '',
        shapeCol: '',
        letter: '',
        letterCol: '',
        orient: '',
      },
      selectedImage: testImage,
      croppedImage: null,
      displayImage: null,
      crop: {
        x: 130,
        y: 50,
        width: 0,
        height: 0,
      },
      zoom: 1,
      aspect: 4 / 3,
      displayCropper: false,
      croppedAreaPixels: null, //result from image crop
      imageList: images,
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
      return {
        formValues  //, curValues
      }
    });
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

  handleSubmit(event) {
    event.preventDefault();
    let formValues = this.state.formValues;
    let annValues = this.state.annValues;

    for (var key in formValues) {
      if (formValues[key] != '') {
        annValues[key] = formValues[key];
        formValues[key] = '';
      }
    }

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

  render() {
    return (
      <div className="Annotations">
        <h1>Annotations</h1>
        <form id="Annotate" onSubmit={this.handleSubmit}>
          <div className="Shape">
            <label className="dualField"> <span className="fieldVal">Shape: </span> <span className="curVal">{this.state.annValues["shape"]}</span>
              <br></br><Select name="shape" isSearchable='true' placeholder="Shape" value={this.state.formValues["shape"]}
                onChange={this.handleSelectChange} options={shapeOptions} styles={selectStyles} />
            </label>
            <label className="dualField"> <span className="fieldVal">Color: </span> <span className="curVal">{this.state.annValues["shapeCol"]}</span>
              <br></br><Select name="shapeCol" isSearchable='true' placeholder="Shape Color" value={this.state.formValues["shapeCol"]}
                onChange={this.handleSelectChange} options={colorOptions} styles={selectStyles} />
            </label>
          </div>

          <div className="Letter">
            <label className="dualField"> <span className="fieldVal">Letter: </span> <span className="curVal">{this.state.annValues["letter"]}</span>
              <br></br><input type="text" name="letter" placeholder="Letter" value={this.state.formValues["letter"]} onChange={this.handleChange} />
            </label>
            <label className="dualField"> <span className="fieldVal">Color: </span> <span className="curVal">{this.state.annValues["letterCol"]}</span>
              <br></br><Select name="letterCol" isSearchable='true' placeholder="Letter Color" value={this.state.formValues["letterCol"]}
                onChange={this.handleSelectChange} options={colorOptions} styles={selectStyles} />
            </label>
          </div>

          <div className="Orient">
            <label className="singField"> <span className="fieldVal">Orientation: </span> <span className="curVal">{this.state.annValues["orient"]}</span>
              <br></br><input type="text" name="orient" placeholder="Orientation" value={this.state.formValues["orient"]} onChange={this.handleChange} />
            </label>
          </div>

          <div className="saveButton">
            <input classNaonChange={this.handleSelectChange} options={colorOptions} styles={selectStyles} me="btn btn-primary" type="submit" value="Save" />
            {this.state.displayCropper ?
              <div>
                <Button variant="primary" primary={true} onClick={this.handleRestore}>Restore</Button>
                <Button variant="primary" primary={true} keyboardFocused={true} onClick={this.handleCropComplete} >Crop</Button>
              </div> : null}

          </div>

        </form>

        <div className="imageList">
          <div className="scroller">
            <List items={this.state.imageList} onItemClick={this.handleListClick} />
          </div>
        </div>

        <div className="imageButton">
          <Button variant="primary" onClick={this.handleOpen}>Toggle image cropper</Button>
          {this.state.displayCropper ?
            <div className="imageCrop">
              <ReactCrop
                image={this.state.displayImage}
                crop={this.state.crop}
                zoom={this.state.zoom}
                aspect={this.state.aspect}
                onCropChange={this.onCropChange}
                onCropComplete={this.onCropComplete}
                onZoomChange={this.onZoomChange}
                maxZoom={1000}
                zoomSpeed={2}
              />
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
}

export default Tagging;