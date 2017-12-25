import React, { Component } from 'react';
import './DarkModal.css'

class DarkModal extends Component {

  render() {
    return (
      <div className="DarkModal">
        <div className={`modal ${this.props.fade ? 'fade' : null}`} id={this.props.myId}>
          <div className={`modal-dialog ${this.props.modalSize}`}>
            <div className="modal-content">
              <div className="modal-header">
                <h5 className="modal-title">{this.props.myTitle}</h5>
                <button className="close" data-dismiss="modal">&times;</button>
              </div>
              <div className="modal-body">
                {this.props.children}
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }
}

export default DarkModal