import React, { Component } from 'react';
import './Navbar.css'
import logo from '../images/vector_logo.svg';

import Option from './Option';

class Navbar extends Component {
  constructor(props) {
    super(props);

  }

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
              handleTab={this.props.handleTab}/>
    ));
  }

  render() {
    return (
      <div className="Navbar">
        <nav className="navbar navbar-expand-sm navbar-dark">
          <a className="navbar-brand" href="/" style={{padding: 0}}>
            <img src={logo} height="40px" alt=""/>
          </a>
          <button className="navbar-toggler" type="button" data-toggle="collapse"
                  data-target="#nav_container" aria-controls="nav_container"
                  aria-expanded="false" aria-label="Toggle navigation">
            <span className="navbar-toggler-icon"></span>
          </button>
          <div className="collapse navbar-collapse" id="nav_container">
            <div className="navbar-nav">
              {this.renderOptions()}
            </div>
            <button className={`btn btn-sm align-middle btn-outline-light ${!this.props.interopBtnEnabled ? 'disabled' : null}`}
                    id="interop_btn" onClick={this.props.interopBtnClick}>{this.props.interopBtnText}</button>
          </div>
        </nav>
      </div>
    );
  }
}

export default Navbar;