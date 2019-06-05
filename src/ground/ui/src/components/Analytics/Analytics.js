import React, { Component } from 'react';
import { connect } from 'react-redux';
import {
  Button, Container, Row, Col,ButtonGroup,
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
    recording: state.telemetry.recording,
    telemetryData: state.telemetry.telemetryData,
  };
}

var loadedTelemetry;
//var telemetryData = [];
//var recording = false;
//var usingLoaded = false;
//var isPaused = false;

const mapDispatchToProps = {
  togglePlayback: function() {
    return {type: 'TOGGLE_PLAYBACK'}
  },
  playback: function(telemetry) {
    return {type: 'PLAYBACK', payload: telemetry}
  },
  loadInteropData: function(data) {
    return {type: 'INTEROP_DATA', payload: data}
  },
  toggleRecording: function() {
    return {type: 'TOGGLE_RECORD'}
  }
}; //TODO Make action for recording telemetry, and action for starting playback


class Analytics extends Component {
  state = {
    usingLoaded: false,
    isPaused: false
  }

  constructor(props) {
    super(props);
    this.handleSubmit = this.handleSubmit.bind(this);
    this.fileInput = React.createRef();
  }

  // componentDidUpdate(prevProps) {
  //   if (this.props.recording && prevProps.telemetry !== this.props.telemetry) {
  //     telemetryData.push(this.props.telemetry)
  //   }
  // }
  render () {
    return (  
      <Container className="Analytics">
        <Row><Col className="logo"><UasLogo/></Col></Row>
        
        <Row>
        <div id="telemetry">
        Telemetry: {JSON.stringify(this.props.telemetry, null, 2)} 
        </div>
        </Row>

        <br />
        <br />

          <Row>
            <Col>
            <Button size="lg" color={this.props.recording ? "danger" : "secondary"} onClick={this.toggleRecord}> 
              {this.props.recording ? "Save" : "Record"}
            </Button>
            </Col>
          </Row>
          <Row><Col>Telemetry States Recorded: <b>{this.props.telemetryData.length}</b></Col></Row>
          

          <br />
            {/* <button onClick={this.handleSubmit}>
              Load Telemetry File / Unload File
            </button> */}
          <Row>
            <Col>
            <Button color={!this.state.usingLoaded ?  "secondary" : this.state.isPaused ? "success" : "info"} onClick={this.runLoaded}>
              {!this.state.usingLoaded ?  "Run Loaded Telemetry" : this.state.isPaused ? "Resume" : "Pause"}
            </Button>
            </Col>
          </Row>

          <br />

          <Row>
            <Col>
            <ButtonGroup>
            <Button color="secondary" onClick={this.saveInteropMission}>
              Save Current Interop Mission
            </Button>
            
            <Button color="primary" onClick={this.loadInteropMission}>
              Load Interop Mission
            </Button>
            </ButtonGroup>
            </Col>
          </Row>
        

        <br />
        <Row>
          <Col>
          <h2>File Browser</h2>
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
    if(this.fileInput.current.files.length > 0){
      reader.readAsText(this.fileInput.current.files[0]);
    }
  }

  downloadTelemetry() {
    downloadToBrowser("telemetry.json", JSON.stringify(this.props.telemetryData,null,2));
  }

  toggleRecord = () => {
    
    if (this.props.recording){
      this.downloadTelemetry();
      var i
      var l = this.props.telemetryData.length
      for (i = 0; i < l; i++){
        this.props.telemetryData.pop()
      }
    }
    this.props.toggleRecording();
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
    if (!this.state.usingLoaded){
      this.props.togglePlayback();
      var reader = new FileReader()
      reader.onload = (e) => {
        //return e.target.result
        loadedTelemetry = JSON.parse(e.target.result)
        this.state.usingLoaded = true
        //console.log(loadedTelemetry)
        var i = 0
        var intervalID = setInterval(() => {
          if(this.state.isPaused){
            
          }else{
            this.props.playback(loadedTelemetry[i])
            i = i + 1
            if (i > loadedTelemetry.length) {
              this.state.usingLoaded = false
              clearInterval(intervalID)
              this.props.togglePlayback()
            }
          }
        }, 250)
        
      }
        //if (reader.readyState == FileReader.EMPTY){
        //  reader.abort()
        //}else{
      //console.log(reader.readyState)
      if(this.fileInput.current.files.length > 0){
        reader.readAsText(this.fileInput.current.files[0]);
      }
      
        //}
    }else{
      this.setState({isPaused: !(this.state.isPaused)});
    }
  }

  stringifyTelemetry = string => {
    if (!(this.state.usingLoaded)) {
      return JSON.stringify(this.props.telemetry)
    }else{
      return JSON.stringify(this.loadedTelemetry)
    }
  }

}

export default connect(mapStateToProps, mapDispatchToProps)(Analytics);