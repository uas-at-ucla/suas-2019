import React, { Component } from "react";
import "./Settings.css";

class Settings extends Component {
  constructor(props) {
    super(props);
    this.state = {
      ip_address: [127, 0, 0, 1, 8000],
    };
    this.interopButtonPressed = this.interopButtonPressed.bind(this);
  }

  renderIPAddress() {
    return (
      <div className="ip_address">
        <form onSubmit={this.interopButtonPressed}>
          <div className="row">
            <div className="col-md-2 offset-md-4">
              <div className="row">
                <input type="text" ref="quad_one"
                       placeholder={this.state.ip_address[0]}>
                </input>
                <p>.</p>
                <input type="text" ref="quad_two"
                       placeholder={this.state.ip_address[1]}>
                </input>
                <p>.</p>
                <input type="text" ref="quad_three"
                       placeholder={this.state.ip_address[2]}>
                </input>
                <p>.</p>
                <input type="text" ref="quad_four"
                       placeholder={this.state.ip_address[3]}>
                </input>
                <p>:</p>
                <input type="text" ref="port"
                       placeholder={this.state.ip_address[4]}>
                </input>
              </div>
            </div>
            <div className="text-left col-md-4">
              <button className={`btn btn-sm align-middle btn-outline-dark ${
              !this.props.appState.interopBtnEnabled ? "disabled" : null}`}
                      id="interop_btn">
                {this.props.appState.interopBtnText}
              </button>
            </div>
          </div>
        </form>
      </div>
    );
  }

  interopButtonPressed(e) {
    e.preventDefault();

    // Either IPv4 is 127.0.0.1:8000 or new values
    var quad_one = ( this.refs.quad_one.value === "" ) ?
                   127 : this.refs.quad_one.value;
    var quad_two = ( this.refs.quad_two.value === "" ) ?
                   0 : this.refs.quad_two.value;
    var quad_three = ( this.refs.quad_three.value === "" ) ?
                     0 : this.refs.quad_three.value;
    var quad_four = ( this.refs.quad_four.value === "" ) ?
                    1 : this.refs.quad_four.value;
    var port = ( this.refs.port.value === "" ) ?
                    8000 : this.refs.port.value;

    var new_ip_address = [ quad_one, quad_two, quad_three, quad_four, port ];
    console.log(new_ip_address);

    this.props.setAppState({ interopBtnText: "Connecting..." });
    this.props.setAppState({ interopBtnEnabled: false });
    this.props.socketEmit("connect_to_interop");
  };

  render() {
    return (
      <div className="Settings">
        <div className="text-center">
          <h1>Settings</h1>
        </div>
        {this.renderIPAddress()}
      </div>
    );
  }
}




export default Settings;