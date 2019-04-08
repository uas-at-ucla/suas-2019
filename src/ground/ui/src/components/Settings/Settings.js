import React, { Component } from "react";
import { connect } from "react-redux";
import {
  Container,
  Button,
  Col,
  InputGroup,
  InputGroupAddon,
  InputGroupText,
  Input,
  Row
} from "reactstrap";
import { Marker } from "react-google-maps";

import "./Settings.css";
import GoogleMap from "components/Utils/GoogleMap/GoogleMap";
import settingsActions from "redux/actions/settingsActions";

const mapStateToProps = state => {
  return {
    settings: state.settings,
    interopData: state.mission.interopData
  };
};

const mapDispatchToProps = settingsActions;

class Settings extends Component {
  handleChange = event => {
    this.props.updateSettings({ [event.target.name]: event.target.value });
  };

  connectToInterop = event => {
    this.props.connectToInterop(
      this.props.settings.interopIp,
      this.props.settings.interopUsername,
      this.props.settings.interopPassword
    );
  };

  connectToGndServer = event => {
    this.props.connectToGndServer();
  };

  handleClickedMap = event => {
    this.props.updateSettings({ antennaPos: {lat: event.latLng.lat(), lng: event.latLng.lng()} });
  };

  render() {
    let connectedGroundServer = this.props.settings.gndServerConnected ? this.props.settings.connectedGndServerIp : "NONE";
    let connectedInteropServer = this.props.interopData ? this.props.interopData.ip : "NONE";
    return (
      <Container className="Settings">
        <Row>
          <Col>
            <InputGroup>
              <InputGroupAddon addonType="prepend">Ground IP</InputGroupAddon>
              <Input
                value={this.props.settings.gndServerIp}
                name="gndServerIp"
                type="text"
                placeholder="0"
                onChange={this.handleChange}
              />
            </InputGroup>
          </Col>
          <Col>
            <Button color="primary" className="connect-btn" onClick={this.connectToGndServer}>
              Connect To Ground Server
            </Button>
            <span>Connected To: {connectedGroundServer}</span>
          </Col>
        </Row>
        <br />
        <Row>
          <Col>
            <InputGroup>
              <InputGroupAddon addonType="prepend">
                Interop Username
              </InputGroupAddon>
              <Input
                value={this.props.settings.interopUsername}
                name="interopUsername"
                type="text"
                placeholder="Enter username..."
                onChange={this.handleChange}
              />
            </InputGroup>
          </Col>
          <Col>
            <InputGroup>
              <InputGroupAddon addonType="prepend">
                Interop Password
              </InputGroupAddon>
              <Input
                value={this.props.settings.interopPassword}
                name="interopPassword"
                type="text"
                placeholder="Enter password..."
                onChange={this.handleChange}
              />
            </InputGroup>
          </Col>
        </Row>
        <br />
        <Row>
          <Col>
            <InputGroup>
              <InputGroupAddon addonType="prepend">Interop IP</InputGroupAddon>
              <Input
                value={this.props.settings.interopIp}
                name="interopIp"
                type="text"
                placeholder={this.props.settings.interopIp}
                onChange={this.handleChange}
                maxLength="18"
              />
            </InputGroup>
          </Col>
          <br />
          <Col>
            <Button color="success" className="connect-btn" onClick={this.connectToInterop}>
              Connect To Interop
            </Button>
            <span>Connected To: {connectedInteropServer}</span>
          </Col>
        </Row>
        <br />

        <Row>
          <Col>
            <h2>File Browser</h2>
            <Input type="file" />
            <br />
          </Col>
          <br />
          <Col>
            <h2>Antenna Tracker Location:</h2>
            <p>
              {this.props.settings.antennaPos.lat}° , {this.props.settings.antennaPos.lng}°
            </p>
            <div style={{ height: "350px", width: "500px" }}>
              <GoogleMap
                defaultZoom={17}
                defaultCenter={{ lat: 34.0689, lng: -118.4452 }}
                defaultMapTypeId="satellite"
                defaultOptions={{
                  disableDefaultUI: true,
                  disableDoubleClickZoom: true,
                  scaleControl: true
                }}
                onDblClick={this.handleClickedMap}
              >
                <Marker
                  position={this.props.settings.antennaPos}
                />
              </GoogleMap>
            </div>
          </Col>
        </Row>
      </Container>
    );
  }
}

export default connect(
  mapStateToProps,
  mapDispatchToProps
)(Settings);
