import React, { Component } from 'react';
import DarkModal from '../DarkModal/DarkModal'

class Panel extends Component {

  render() {
    let header = null;
    if (this.props.myTitle) {
      header = (
        <h4 className="card-header">
          {this.props.myTitle}
          <button className="panel-expand" data-toggle="modal" data-target={`#${this.props.myId}`}>
            <i className="fa fa-expand" aria-hidden="true"></i>
          </button>
        </h4>
      );
    }

    return (
      <div className="Panel">
        <div className="card text-white">
          {header}
          <div className="card-body">
            {this.props.children}
          </div>
        </div>

        <DarkModal myId={this.props.myId} myTitle={this.props.myTitle} modalSize="modal-lg" fade>
          {this.props.children}
        </DarkModal>
      </div>
    );
  }
}

export default Panel
