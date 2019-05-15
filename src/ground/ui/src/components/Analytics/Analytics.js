import React, { Component } from 'react';
import { connect } from 'react-redux';
import {
  Button, Container, Row, Col,
  InputGroup, InputGroupAddon, InputGroupText, Input,
  InputGroupButtonDropdown, DropdownToggle, DropdownMenu, DropdownItem
} from "reactstrap";

import './Analytics.css';
import downloadToBrowser from 'utils/downloadToBrowser';

import UasLogo from "components/utils/UasLogo/UasLogo";


const mapStateToProps = state => {
  return {
    telemetry: state.telemetry.droneTelemetry,
    playback: state.telemetry.playback,
    interopData: state.mission.interopData,
  };
}

var loadedTelemetry;
var telemetryData = [];
//var recording = false;
var usingLoaded = false;
var isPaused = false;

const mapDispatchToProps = {
  togglePlayback: function() {
    return {type: 'TOGGLE_PLAYBACK'}
  },
  playback: function(telemetry) {
    return {type: 'PLAYBACK', payload: telemetry}
  },
  loadInteropData: function(data) {
    return {type: 'INTEROP_DATA', payload: data}
  }
}; //TODO Make action for recording telemetry, and action for starting playback


class Analytics extends Component {
  state = {
    recording: false
  }

  constructor(props) {
    super(props);
    this.handleSubmit = this.handleSubmit.bind(this);
    this.fileInput = React.createRef();
  }

  componentDidUpdate(prevProps) {
    if (this.state.recording && prevProps.telemetry !== this.props.telemetry) {
      telemetryData.push(this.props.telemetry)
    }
  }
  render () {
    return (  
      <Container className="Analytics">
        <Row><Col></Col><Col className="logo"><UasLogo/></Col></Row>
        
        <Row>
        <div id="telemetry">
        Telemetry: {JSON.stringify(this.props.telemetry)} 
        </div>
        </Row>

        <br />
        <br />

          <Row>
            <Col>
            <Button color={this.state.recording ? "danger" : "success"} onClick={this.toggleRecord}>
              Record!!! / Stop Recording (and save to file)!!
            </Button>
            </Col>
          </Row>
          <Row><Col>Telemetry States Recorded: <b>{telemetryData.length}</b></Col></Row>
          

          <br />
            {/* <button onClick={this.handleSubmit}>
              Load Telemetry File / Unload File
            </button> */}
          <Row>
            <Col>
            <Button onClick={this.runLoaded}>
              Run Loaded Telemetry / Pause Loaded Telemetry
            </Button>
            </Col>
          </Row>

          <br />

          <Row>
            <Col>
            <Button onClick={this.saveInteropMission}>
              Save Current Interop Mission
            </Button>
            </Col>
            <Col>
            <Button onClick={this.loadInteropMission}>
              Load Interop Mission / Unload File
            </Button>
            </Col>
          </Row>
        

        <br />

        <Row>
          <Col>
          <input type="file" ref={this.fileInput} />
          </Col>
        </Row>

        </Container>
    );
  }

  saveInteropMission = () => {
    //TODO: add the interop data to mapStateToProps (see Map.js)
    //Save it to a file using JSON.stringify and downloadToBrowser
    //console.log(this.props.interopData)
    downloadToBrowser("interop.json", JSON.stringify(this.props.interopData, null, 2))
  }

  loadInteropMission = () => {
    //TODO load JSON data from a file
    //this.props.loadInteropData({ type: 'INTEROP_DATA', payload: data });
    
    var reader = new FileReader()
    reader.onload = (e) => {
      let data = JSON.parse(e.target.result)
      console.log(data)
      this.props.loadInteropData(data)
    }
    if (reader.readyState == FileReader.EMPTY){
      reader.abort()
    }else{
      reader.readAsText(this.fileInput.current.files[0]);
    }
  }

  downloadTelemetry() {
    downloadToBrowser("telemetry.json", JSON.stringify(telemetryData,null,2));
  }

  toggleRecord = () => {
    this.state.recording = !(this.state.recording);
    if (!this.state.recording){
      this.downloadTelemetry();
      var i
      var l = telemetryData.length
      for (i = 0; i < l; i++){
        telemetryData.pop()
      }
    }
  }

  handleSubmit(event) {
    event.preventDefault();
    alert(
      `Selected file - ${
        this.fileInput.current.files[0].name
      }`
    );
  }

  runLoaded = () => {
    if (!usingLoaded){
      this.props.togglePlayback();
      var reader = new FileReader()
      reader.onload = (e) => {
        //return e.target.result
        loadedTelemetry = JSON.parse(e.target.result)
        usingLoaded = true
        //console.log(loadedTelemetry)
        var i = 0
        var intervalID = setInterval(() => {
          if(isPaused){
            
          }else{
            this.props.playback(loadedTelemetry[i])
            i = i + 1
            if (i > loadedTelemetry.length) {
              usingLoaded = false
              clearInterval(intervalID)
              this.props.togglePlayback()
            }
          }
        }, 250)
        
        }
        if (reader.readyState == FileReader.EMPTY){
          reader.abort()
        }else{
          reader.readAsText(this.fileInput.current.files[0]);
        }
    }else{
      isPaused = !isPaused
    }
  }

  stringifyTelemetry = string => {
    if (!(this.usingLoaded)) {
      return JSON.stringify(this.props.telemetry)
    }else{
      return JSON.stringify(this.loadedTelemetry)
    }
  }

}

export default connect(mapStateToProps, mapDispatchToProps)(Analytics);