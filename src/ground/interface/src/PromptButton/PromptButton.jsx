import React, { Component } from "react";
import "./PromptButton.css";

const SUPER_SECRET_CODE = "spinny"

class PromptButton extends Component {

  state = {input: ""}

  componentDidMount() {
    window.$(`#${this.props.id}-modal`).on('shown.bs.modal', () => {
      this.refs.code_input.focus();
    });

    window.$(`#${this.props.id}-modal`).on('hidden.bs.modal', () => {
      this.setState({input: ""});
    });
  }

  render() {
    return (
      <div className="PromptButton" id={this.props.id}>
        <button
          className={this.props.className}
          data-toggle="modal"
          data-target={`#${this.props.id}-modal`}
        >
          {this.props.children}
        </button>

        <div className="modal fade" id={`${this.props.id}-modal`}>
          <div className="modal-dialog">
            <div className="modal-content">
              <div className="modal-header">
                <h5 className="modal-title">Enter Confirmation Code</h5>
                <button className="close" data-dismiss="modal">
                  &times;
                </button>
              </div>
              <div className="modal-body">
                <input ref="code_input" type="password" className="form-control" value={this.state.input} onChange={e => this.setState({input: e.target.value})} onKeyUp={e => e.keyCode === 13 ? this.refs.btn.click() : null }/>
                <button
                  ref = "btn"
                  data-dismiss="modal"
                  className={`${this.props.className} btn-lg`}
                  onClick={this.props.onClick}
                  disabled={this.state.input !== SUPER_SECRET_CODE}
                >
                  {this.props.children}
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }
}

export default PromptButton;
