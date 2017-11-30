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
              app={this.props.app}/>
    ));
  }

  render() {
    return (
      <div className="Navbar">
          <div className="nav">
            {this.renderOptions()}
            <button className={`btn btn-sm align-middle btn-outline-light ${!this.props.app.state.interopBtnEnabled ? 'disabled' : null}`}
              id="interop_btn" onClick={this.props.app.connectToInterop}>{this.props.app.state.interopBtnText}</button>
          </div>
      </div>
    );
  }
}

export default Navbar;