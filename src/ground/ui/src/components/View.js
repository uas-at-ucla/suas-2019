import React, { Component } from 'react';
import { connect } from 'react-redux';
import { Button } from 'reactstrap';

import { add } from '../actions/counterActions';

const mapStateToProps = state => {
  return {
    value: state.getIn(['counter', 'value'])
  };
};

class View extends Component {
  render() {
    return (
      <div className="View">
        <div>The magic number is {this.props.value}</div>
        <Button onClick={this.add}>Add!</Button>
      </div>
    );
  }

  add = () => {
    this.props.dispatch(add(1));
  }
}

export default connect(mapStateToProps)(View);
