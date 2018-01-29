import React, { Component } from "react";
import "./Settings.css";

class Settings extends Component {
  constructor(props) {
    super(props);
    this.state = {
      ip_address: [127, 0, 0, 1, 8000],
    };
  }

  renderIPAddress() {
    return (
      <div className="ip_address">
        <form>
          <div className="row">
            <div className="col-md-2 offset-md-4">
              <div className="row">
                <input type="text" name="quad-one"
                       placeholder={this.state.ip_address[0]}>
                </input>
                <p>.</p>
                <input type="text" name="quad-two"
                       placeholder={this.state.ip_address[1]}>
                </input>
                <p>.</p>
                <input type="text" name="quad-three"
                       placeholder={this.state.ip_address[2]}>
                </input>
                <p>.</p>
                <input type="text" name="quad-four"
                       placeholder={this.state.ip_address[3]}>
                </input>
                <p>:</p>
                <input type="text" name="port"
                       placeholder={this.state.ip_address[4]}>
                </input>
              </div>
            </div>
            <div className="text-left col-md-4">
              <button className={`btn btn-sm align-middle btn-outline-dark ${
              !this.props.appState.interopBtnEnabled ? "disabled" : null}`}
                      id="interop_btn"
                      onClick={this.connectToInterop}>
                {this.props.appState.interopBtnText}
              </button>
            </div>
          </div>
        </form>
      </div>
    );
  }

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

  connectToInterop = () => {
    this.props.setAppState({ interopBtnText: "Connecting..." });
    this.props.setAppState({ interopBtnEnabled: false });
    this.props.socketEmit("connect_to_interop");
  };
}




export default Settings;