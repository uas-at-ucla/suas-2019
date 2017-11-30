import React, { Component } from 'react';
import './Navbar.css'

import Option from './Option';

class Navbar extends Component {

  // Display tab titles here
  getOptions() {
    return [
      { _id: 1, section: 'Home' },
      { _id: 2, section: 'Analytics' },
    ];
  }

  renderOptions() {
    return this.getOptions().map((option) => (
      <Option key={option._id} option={option}
              appState={this.props.appState} setAppState={this.props.setAppState}/>
    ));
  }

  render() {
    return (
      <div className="Navbar">
          <div className="nav">
            {this.renderOptions()}
            <button className={`btn btn-sm align-middle btn-outline-light ${!this.props.appState.interopBtnEnabled ? 'disabled' : null}`}
                    id="interop_btn" onClick={this.connectToInterop}>{this.props.appState.interopBtnText}</button>
          </div>
      </div>
    );
  }

  connectToInterop = () => {
    this.props.setAppState({interopBtnText: "Connecting..."});
    this.props.setAppState({interopBtnEnabled: false});
    this.props.socketEmit('connect_to_interop');
  }
}

export default Navbar;