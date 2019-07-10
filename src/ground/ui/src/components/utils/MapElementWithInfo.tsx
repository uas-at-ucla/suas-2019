import React, { Component } from "react";
import { InfoWindow } from "react-google-maps";

interface Props {
  isOpen: { [key: string]: boolean };
  toggleOpen: (id: string) => void;
  name: string;
  infoPosition?: any;
  Element: typeof Component;
  [key: string]: any; //other props for the map element
}

class MapElementWithInfo extends Component<Props> {
  private toggleOpen = () => {
    this.props.toggleOpen(this.props.name);
  };

  public render() {
    let info = this.props.isOpen[this.props.name] ? (
      <InfoWindow
        onCloseClick={this.toggleOpen}
        position={this.props.infoPosition}
      >
        <div className="map-infobox">{this.props.children}</div>
      </InfoWindow>
    ) : null;

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
