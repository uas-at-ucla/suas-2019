import React, { Component } from "react";

// Option Component
class Option extends Component {
  handleOption = e => {
    e.preventDefault();
    let newState = { optionSelected: this.props.option.section };
    if (this.props.option.section === "Control") {
      newState.followDrone = true;
    }
    this.props.setAppState(newState);
  };

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
