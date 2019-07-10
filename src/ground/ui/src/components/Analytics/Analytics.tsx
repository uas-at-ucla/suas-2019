import React, { Component } from "react";
import { connect } from "react-redux";
import { Button, ButtonGroup, Col, Container, Row, Progress } from "reactstrap";

import UasLogo from "components/utils/UasLogo/UasLogo";
import { AppState } from "redux/store";
import * as analyticsActions from "redux/actions/analyticsActions";
import downloadToBrowser from "utils/downloadToBrowser";
import "./Analytics.css";

const mapStateToProps = (state: AppState) => {
  return {
    telemetry: state.telemetry.droneTelemetry,
    ugvStatus: state.telemetry.ugvStatus,
    interopData: state.mission.interopData,
    recording: state.telemetry.recording,
    telemetryData: state.telemetry.telemetryData
  };
};

let loadedTelemetry: any;
// var telemetryData = [];
// var recording = false;
// var usingLoaded = false;
// var isPaused = false;

const mapDispatchToProps = analyticsActions;

type Props = ReturnType<typeof mapStateToProps> & (typeof mapDispatchToProps);

class Analytics extends Component<Props> {
  public state = {
    isPaused: false
  };
  private usingLoaded: boolean = false;
  private fileInput: React.RefObject<HTMLInputElement> = React.createRef();

  // componentDidUpdate(prevProps) {
  //   if (this.props.recording && prevProps.telemetry !== this.props.telemetry) {
  //     telemetryData.push(this.props.telemetry)
  //   }
  // }
  public render() {
    return (
      <Container className="Analytics">
        <Row>
          <Col className="logo">
            <UasLogo />
          </Col>
        </Row>

        <Row>
          <div id="telemetry">
            Telemetry: {JSON.stringify(this.props.telemetry, null, 2)}
          </div>
        </Row>

        <br />
        <br />

        <Row>
          <Col>
            <Button
              size="lg"
              color={this.props.recording ? "danger" : "secondary"}
              onClick={this.toggleRecord}
            >
              {this.props.recording ? "Save" : "Record"}
            </Button>
          </Col>
        </Row>
        <Row>
          <Col>
            Telemetry States Recorded: <b>{this.props.telemetryData.length}</b>
          </Col>
        </Row>

        <br />
        {/* <button onClick={this.handleSubmit}>
              Load Telemetry File / Unload File
            </button> */}
        <Row>
          <Col>
            <Button
              color={
                !this.usingLoaded
                  ? "secondary"
                  : this.state.isPaused
                  ? "success"
                  : "info"
              }
              onClick={this.runLoaded}
            >
              {!this.usingLoaded
                ? "Run Loaded Telemetry"
                : this.state.isPaused
                ? "Resume"
                : "Pause"}
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

        <br />
        <Row>
          <div id="ugv-status">
            UGV Status: {JSON.stringify(this.props.ugvStatus, null, 2)}
          </div>
        </Row>
      </Container>
    );
  }

  private saveInteropMission = () => {
    // TODO: add the interop data to mapStateToProps (see Map.js)
    // Save it to a file using JSON.stringify and downloadToBrowser
    // console.log(this.props.interopData)
    downloadToBrowser(
      "interop.json",
      JSON.stringify(this.props.interopData, null, 2)
    );
  };

  private loadInteropMission = () => {
    // TODO load JSON data from a file
    // this.props.loadInteropData({ type: 'INTEROP_DATA', payload: data });

    let reader = new FileReader();
    reader.onload = () => {
      const data = JSON.parse(reader.result as string);
      console.log(data);
      this.props.loadInteropData(data);
    };
    if (
      this.fileInput.current &&
      this.fileInput.current.files &&
      this.fileInput.current.files.length > 0
    ) {
      reader.readAsText(this.fileInput.current.files[0]);
    }
  };

  private downloadTelemetry() {
    downloadToBrowser(
      "telemetry.json",
      JSON.stringify(this.props.telemetryData, null, 2)
    );
  }

  private toggleRecord = () => {
    if (this.props.recording) {
      this.downloadTelemetry();
      let i;
      let l = this.props.telemetryData.length;
      for (i = 0; i < l; i++) {
        this.props.telemetryData.pop();
      }
    }
    this.props.toggleRecording();
  };

  private handleSubmit = (event: any) => {
    event.preventDefault();
    if (this.fileInput.current && this.fileInput.current.files)
      alert(`Selected file - ${this.fileInput.current.files[0].name}`);
  };

  private runLoaded = () => {
    if (!this.usingLoaded) {
      this.props.togglePlayback();
      let reader: FileReader = new FileReader();
      reader.onload = e => {
        loadedTelemetry = JSON.parse(reader.result as string);
        this.usingLoaded = true;
        // console.log(loadedTelemetry)
        let i = 0;
        let intervalID = setInterval(() => {
          if (this.state.isPaused) {
          } else {
            this.props.playback(loadedTelemetry[i]);
            i = i + 1;
            if (i > loadedTelemetry.length) {
              this.usingLoaded = false;
              clearInterval(intervalID);
              this.props.togglePlayback();
            }
          }
        }, 250);
      };
      // if (reader.readyState == FileReader.EMPTY){
      //  reader.abort()
      // }else{
      // console.log(reader.readyState)
      if (
        this.fileInput.current &&
        this.fileInput.current.files &&
        this.fileInput.current.files.length > 0
      ) {
        reader.readAsText(this.fileInput.current.files[0]);
      }

      // }
    } else {
      this.setState({ isPaused: !this.state.isPaused });
    }
  };

  // private stringifyTelemetry = string => {
  //   if (!this.usingLoaded) {
  //     return JSON.stringify(this.props.telemetry);
  //   } else {
  //     return JSON.stringify(this.loadedTelemetry);
  //   }
  // };
}

export default connect(
  mapStateToProps,
  mapDispatchToProps
)(Analytics);
