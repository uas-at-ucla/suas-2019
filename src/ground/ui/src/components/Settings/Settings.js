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
    settings: state.settings
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
      this.props.settings.username,
      this.props.settings.password
    );
    alert("Connected to: " + this.props.settings.connected_ip);
  };

  handleClickedMap = event => {
    this.props.updateSettings({ lat: event.latLng.lat(), lng: event.latLng.lng() });
  };

  render() {
    return (
      <Container className="Settings">
        <h1>Settings</h1>
        <h2>Connected To: {this.props.settings.ip}</h2>
        <Row>
          <Col>
            <InputGroup>
              <InputGroupAddon addonType="prepend">
                Interop Username
              </InputGroupAddon>
              <Input
                value={this.props.settings.username}
                name="username"
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
                value={this.props.settings.password}
                name="password"
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
            <Button color="danger" onClick={this.connectToInterop}>
              Connect To Interop
            </Button>
          </Col>
        </Row>

        <br />
        <Col>
          <InputGroup>
            <InputGroupAddon addonType="prepend">Ground IP</InputGroupAddon>
            <Input
              value={this.props.settings.groundIp}
              name="groundIp"
              type="text"
              placeholder="0"
              onChange={this.handleChange}
            />
          </InputGroup>
        </Col>
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
              {this.props.settings.lat}° , {this.props.settings.lng}°
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
                  position={{ lat: this.props.settings.lat, lng: this.props.settings.lng }}
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
