import React, { Component } from 'react';
import { InfoWindow } from 'react-google-maps';

class MapElementWithInfo extends Component {
  toggleOpen = () => {
    this.props.toggleOpen(this.props.name);
  };

  render() {
    let info = this.props.isOpen[this.props.name] ?
      <InfoWindow onCloseClick={this.toggleOpen} position={this.props.infoPosition}>
        <div className="map-infobox">
          {this.props.children}
        </div>
      </InfoWindow> 
    : null;

    return (
      <span>
        <this.props.Element {...this.props} onClick={this.toggleOpen}>
          {!this.props.infoPosition ? info : null}
        </this.props.Element>
        {this.props.infoPosition ? info : null}
      </span>
    );
  }
}

export default MapElementWithInfo;