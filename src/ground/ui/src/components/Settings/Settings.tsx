import React, { Component, ChangeEvent, MouseEvent } from "react";
import { connect } from "react-redux";
import {
  Button,
  Container,
  Row,
  Col,
  InputGroup,
  InputGroupAddon,
  Input,
  InputGroupButtonDropdown,
  DropdownToggle,
  DropdownMenu,
  DropdownItem
} from "reactstrap";
import { Marker } from "react-google-maps";

import "./Settings.css";
import GoogleMap from "components/utils/GoogleMap/GoogleMap";
import UasLogo from "components/utils/UasLogo/UasLogo";
import * as settingsActions from "redux/actions/settingsActions";
import * as genericActions from "redux/actions/genericActions";
import { AppState } from "redux/store";

const defaultMapCenter = { lat: 34.0689, lng: -118.4452 };

const mapStateToProps = (state: AppState) => {
  return {
    settings: state.settings,
    interopData: state.mission.interopData
  };
};

const mapDispatchToProps = { ...settingsActions, ...genericActions };

type Props = ReturnType<typeof mapStateToProps> & (typeof mapDispatchToProps);

class Settings extends Component<Props> {
  public state = {
    gndServerDropdown: false,
    interopDropdown: false,
    compLocDropdown: false
  };

  private toggleGndServerDropdown = () =>
    this.setState({ gndServerDropdown: !this.state.gndServerDropdown });
  private toggleInteropDropdown = () =>
    this.setState({ interopDropdown: !this.state.interopDropdown });
  private toggleCompLocDropdown = () =>
    this.setState({ compLocDropdown: !this.state.compLocDropdown });

  private handleChange = (event: ChangeEvent<HTMLInputElement>) => {
    this.props.updateSettings(({
      [event.target.name]: event.target.value
    } as unknown) as AppState["settings"]);
  };

  private handleSelect = (event: MouseEvent<HTMLElement>) => {
    console.log(event.target);
    let name = event.currentTarget.dataset.name as string;
    this.props.updateSettings(({
      [name]: (event.target as HTMLElement).innerText
    } as unknown) as AppState["settings"]);
  };

  private connectToInterop = () => {
    this.props.connectToInterop(
      this.props.settings.interopIp,
      this.props.settings.interopUsername,
      this.props.settings.interopPassword,
      this.props.settings.interopMissionId
    );
  };

  private connectToGndServer = () => {
    this.props.connectToGndServer();
  };

  private configureTrackyPos = () => {
    this.props.configureTrackyPos(this.props.settings.antennaPos);
  };

  private configureUgvDest = () => {
    this.props.configureUgvDest(this.props.settings.antennaPos); // temporary, but convenient to use the same map
  };

  private handleClickedMap = (event: any) => {
    this.props.updateSettings(({
      antennaPos: { lat: event.latLng.lat(), lng: event.latLng.lng() }
    } as unknown) as AppState["settings"]);
  };

  public render() {
    let connectedGroundServer = this.props.settings.gndServerConnected
      ? this.props.settings.connectedGndServerIp
      : "NONE";
    let connectedInteropServer = this.props.interopData
      ? this.props.interopData.ip
      : "NONE";
    return (
      <Container className="Settings">
        <Row>
          <Col className="logo">
            <UasLogo />
          </Col>
        </Row>
        <Row>
          <Col>
            <InputGroup>
              <InputGroupButtonDropdown
                addonType="prepend"
                isOpen={this.state.gndServerDropdown}
                toggle={this.toggleGndServerDropdown}
              >
                <DropdownToggle caret>Ground Server IP</DropdownToggle>
                <DropdownMenu
                  data-name="gndServerIp"
                  onClick={this.handleSelect}
                >
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
            <Button
              color="primary"
              className="connect-btn"
              onClick={this.connectToGndServer}
            >
              Connect To Ground Server
            </Button>
          </Col>
          <Col className="connect-status">
            <span>
              Connected to <b>{connectedGroundServer}</b>
            </span>
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
              <InputGroupAddon addonType="prepend">Mission ID</InputGroupAddon>
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
                addonType="prepend"
                isOpen={this.state.interopDropdown}
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
            <Button
              color="success"
              className="connect-btn"
              onClick={this.connectToInterop}
            >
              Connect To Interop
            </Button>
          </Col>
          <Col className="connect-status">
            <span>
              Connected to <b>{connectedInteropServer}</b>
            </span>
          </Col>
        </Row>
        <br />
        <Row>
          <Col>
            <Button onClick={this.props.logReduxState}>Log Redux State</Button>
            <Button color="danger" onClick={this.props.resetReduxState}>
              [DANGER] Reset Redux State
            </Button>
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
              {this.props.settings.antennaPos.lat}° ,{" "}
              {this.props.settings.antennaPos.lng}°
            </p>
            <Button color="success" onClick={this.configureTrackyPos}>
              Send Position!
            </Button>
            <Button color="danger" onClick={this.configureUgvDest}>
              <b>UGV</b> Target!
            </Button>
            <div style={{ height: "350px", width: "500px" }}>
              <GoogleMap
                defaultZoom={17}
                center={
                  /*if*/ this.props.interopData
                    ? {
                        lat: this.props.interopData.mission.airDropPos.latitude,
                        lng: this.props.interopData.mission.airDropPos.longitude
                      }
                    : /*else*/ defaultMapCenter
                }
                defaultMapTypeId="customTiles"
                defaultOptions={{
                  disableDefaultUI: true,
                  disableDoubleClickZoom: true,
                  scaleControl: true
                }}
                onDblClick={this.handleClickedMap}
              >
                <Marker position={this.props.settings.antennaPos} />
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
