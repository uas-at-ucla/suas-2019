import React, { Component } from 'react';

// Option Componeent
class Option extends Component {

  handleOption = (e) => {
    e.preventDefault();
    this.props.setAppState({optionSelected: this.props.option.section});
  }

  render() {
    return (
      <li>
        <a className="nav-item nav-link" href="/" onClick={this.handleOption}>
          {this.props.option.section}
        </a>
      </li>
    );
  }
}

export default Option;
