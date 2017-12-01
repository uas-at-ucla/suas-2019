import React, { Component } from 'react';

class Panel extends Component {

  render() {
    let header = null;
    if (this.props.title) {
      header = <h4 className="card-header">{this.props.title}</h4>
    }

    return (
      <div className="card text-white">
        {header}
        <div className="card-body">
          {this.props.children}
        </div>
      </div>
    );
  }
}

export default Panel
