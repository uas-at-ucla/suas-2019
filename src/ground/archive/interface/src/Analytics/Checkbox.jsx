import React, { Component } from "react";

class Checkbox extends Component {
  constructor(props) {
    super(props);
    this.state = { checked: true };
    this.toggle = this.toggle.bind(this);
  }
  toggle() {
    this.setState(prevState => {
      this.props.updateVal(this.props.valName, !prevState.checked);
      return { checked: !prevState.checked };
    });
  }
  render() {
    return (
      <input
        type="checkbox"
        checked={this.state.checked}
        onClick={this.toggle}
      />
    );
  }
}
export default Checkbox;
