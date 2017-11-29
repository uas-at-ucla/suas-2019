import React, { Component } from 'react';

// Option Componeent
class Option extends Component {
  constructor(props) {
    super(props);
    this.handleOptions = this.handleOptions.bind(this);
  }

  // Callback function to Parent Component
  handleOptions(e) {
    e.preventDefault();
    this.props.handleTab(this.props.option.section);
  }

  render() {
    return (
      <li>
        <a className="nav-item nav-link" href="/" onClick={this.handleOptions}>
          {this.props.option.section}
          <span className="sr-only"></span>
        </a>
      </li>
    );
  }
}

export default Option;
