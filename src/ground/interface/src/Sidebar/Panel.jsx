import React, { Component } from "react";

class Panel extends Component {
  render() {
    let header = null;
    if (this.props.title) {
      header = (
        <h4>
          {this.props.title}
          <button
            className="panel-expand"
            data-toggle="modal"
            data-target={`#${this.props.id}-modal`}
          >
            <i className="fa fa-expand" aria-hidden="true" />
          </button>
        </h4>
      );
    }

    return (
      <div className="Panel" id={this.props.id}>
        <div className="card text-white">
          <div className="card-body">
            {header}
            {this.props.children}
          </div>
        </div>

        <div className="modal fade" id={`${this.props.id}-modal`}>
          <div className="modal-dialog modal-lg">
            <div className="modal-content">
              <div className="modal-header">
                <h5 className="modal-title">{this.props.title}</h5>
                <button className="close" data-dismiss="modal">
                  &times;
                </button>
              </div>
              <div className="modal-body">{this.props.children}</div>
            </div>
          </div>
        </div>
      </div>
    );
  }
}

export default Panel;
