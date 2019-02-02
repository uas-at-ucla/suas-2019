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
import GoogleMap from "../Utils/GoogleMap/GoogleMap";
import "./Settings.css";
import settingsActions from "../../actions/settingsActions";

const mapStateToProps = redux_state => {
  return {
    redux_settings: redux_state.settings
  };
};

const mapDispatchToProps = settingsActions;

class Settings extends Component {
  constructor(props) {
    super(props);
    this.state = this.props.redux_settings;
  }

  handleChange = event => {
    this.setState({ [event.target.name]: event.target.value });
  };

  handleSubmit = event => {
    alert("Connected to: " + this.state.ip);
    event.preventDefault();
  };

  handleClickedMap = event => {
    this.setState({ lat: event.latLng.lat(), long: event.latLng.lng() });
  };

  render() {
    console.log(this.state);
    return (
      <Container>
        <h1>Settings</h1>
        <h2>Connected To: {this.state.ip}</h2>
        <Row>
          <Col>
            <InputGroup>
              <InputGroupAddon addonType="prepend">
                Interop Username
              </InputGroupAddon>
              <Input
                value={this.state.un}
                name="un"
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
                value={this.state.pw}
                name="pw"
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
                value={this.state.ip}
                name="ip"
                type="text"
                placeholder={this.state.ip}
                onChange={this.handleChange}
                maxLength="18"
              />
            </InputGroup>
          </Col>
          <br />
          <Col>
            <Button color="danger" onClick={this.handleSubmit}>
              Connect To Interop
            </Button>
          </Col>
        </Row>

        <br />
        <Col>
          <InputGroup>
            <InputGroupAddon addonType="prepend">Ground IP</InputGroupAddon>
            <Input
              value={this.state.gip}
              name="gip"
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
          </Col>
          <br />
          <Col>
            <h2>Antenna Tracker Location:</h2>
            <p>
              {this.state.lat}° , {this.state.long}°
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
                  position={{ lat: this.state.lat, lng: this.state.long }}
                />
              </GoogleMap>
            </div>
          </Col>
          <Button onClick={this.updateSettings}>Update Settings</Button>
        </Row>
      </Container>
    );
  }

  updateSettings = () => {
    this.props.updateSettings(this.state);
  };
}

export default connect(
  mapStateToProps,
  mapDispatchToProps
)(Settings);
