import React, { Component } from "react";
import { connect } from "react-redux";
import {
  Button, Container, Row, Col,
  InputGroup, InputGroupAddon, Input,
  InputGroupButtonDropdown, DropdownToggle, DropdownMenu, DropdownItem
} from "reactstrap";
import { Marker } from "react-google-maps";

import "./Settings.css";
import GoogleMap from "components/utils/GoogleMap/GoogleMap";
import UasLogo from "components/utils/UasLogo/UasLogo";
import settingsActions from "redux/actions/settingsActions";

const defaultMapCenter = {lat: 34.0689, lng: -118.4452};

const mapStateToProps = state => {
  return {
    settings: state.settings,
    interopData: state.mission.interopData
  };
};

const mapDispatchToProps = settingsActions;

class Settings extends Component {
  state = { 
    gndServerDropdown: false,
    interopDropdown: false
  };
  toggleGndServerDropdown = () => this.setState({ gndServerDropdown: !this.state.gndServerDropdown });
  toggleInteropDropdown = () => this.setState({ interopDropdown: !this.state.interopDropdown });
  toggleCompLocDropdown = () => this.setState({ compLocDropdown: !this.state.compLocDropdown });

  handleChange = (event) => {
    this.props.updateSettings({ [event.target.name]: event.target.value });
  };

  handleSelect = (event) => {
    console.log(event.target)
    let name = event.target.parentElement.dataset.name;
    if (name) {
      this.props.updateSettings({ [name]: event.target.innerText });
    }
  };

  changeCompLocation = (event) => {
    this.handleSelect(event);
    this.props.changeCompLocation(event.target.innerText);
  }

  connectToInterop = () => {
    this.props.connectToInterop(
      this.props.settings.interopIp,
      this.props.settings.interopUsername,
      this.props.settings.interopPassword,
      this.props.settings.interopMissionId
    );
  };

  connectToGndServer = () => {
    this.props.connectToGndServer();
  };

  configureTrackyPos = () => {
    this.props.configureTrackyPos(this.props.settings.antennaPos);
  }

  configureUgvDest = () => {
    this.props.configureUgvDest(this.props.settings.antennaPos); // temporary, but convenient to use the same map
  }

  handleClickedMap = (event) => {
    this.props.updateSettings({ antennaPos: {lat: event.latLng.lat(), lng: event.latLng.lng()} });
  };

  render() {
    let connectedGroundServer = this.props.settings.gndServerConnected ? this.props.settings.connectedGndServerIp : "NONE";
    let connectedInteropServer = this.props.interopData ? this.props.interopData.ip : "NONE";
    return (
      <Container className="Settings">
        <Row><Col className="logo"><UasLogo/></Col></Row>
        <Row>
          <Col>
            <InputGroup>
              <InputGroupButtonDropdown 
                addonType="prepend" isOpen={this.state.gndServerDropdown} 
                toggle={this.toggleGndServerDropdown}
              >
                <DropdownToggle caret>Ground Server IP</DropdownToggle>
                <DropdownMenu data-name="gndServerIp" onClick={this.handleSelect}>
                  <DropdownItem>localhost:8081</DropdownItem>
                  <DropdownItem>192.168.1.10:8081</DropdownItem>
                </DropdownMenu>
              </InputGroupButtonDropdown>
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
          </Col>
          <Col className="connect-status">
            <span>Connected to <b>{connectedGroundServer}</b></span>
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
                type="password"
                placeholder="Enter password..."
                onChange={this.handleChange}
              />
            </InputGroup>
          </Col>
          <Col>
            <InputGroup>
              <InputGroupAddon addonType="prepend">
                Mission ID
              </InputGroupAddon>
              <Input
                value={this.props.settings.interopMissionId}
                name="interopMissionId"
                type="text"
                placeholder="Provided by the judges"
                onChange={this.handleChange}
              />
            </InputGroup>
          </Col>
        </Row>
        {/* <br /> */}
        <Row>
          <Col>
            <InputGroup>
              <InputGroupButtonDropdown 
                addonType="prepend" isOpen={this.state.interopDropdown} 
                toggle={this.toggleInteropDropdown}
              >
                <DropdownToggle caret>Interop IP</DropdownToggle>
                <DropdownMenu data-name="interopIp" onClick={this.handleSelect}>
                  <DropdownItem>134.209.2.203:8000</DropdownItem>
                </DropdownMenu>
              </InputGroupButtonDropdown>
              <Input
                value={this.props.settings.interopIp}
                name="interopIp"
                type="text"
                placeholder={this.props.settings.interopIp}
                onChange={this.handleChange}
              />
            </InputGroup>
          </Col>
          <br />
          <Col>
            <Button color="success" className="connect-btn" onClick={this.connectToInterop}>
              Connect To Interop
            </Button>
          </Col>
          <Col className="connect-status">
            <span>Connected to <b>{connectedInteropServer}</b></span>
          </Col>
        </Row>
        <br />
        <Row>
          <Col>
            <Button onClick={this.props.logReduxState}>Log Redux State</Button>
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
            <Button color="success" onClick={this.configureTrackyPos}>Send Position!</Button>
            <Button color="danger" onClick={this.configureUgvDest}><b>UGV</b> Target!</Button>
            <div style={{ height: "350px", width: "500px" }}>
              <GoogleMap
                defaultZoom={17}
                center={
                  /*if*/ this.props.interopData ? 
                    {
                      lat: this.props.interopData.mission.airDropPos.latitude,
                      lng: this.props.interopData.mission.airDropPos.longitude
                    } 
                  /*else*/: 
                    defaultMapCenter
                }
                defaultMapTypeId="customTiles"
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
