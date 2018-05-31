import React, { Component } from "react";
import "./Settings.css";

class Settings extends Component {
  constructor(props) {
    super(props);
    this.state = {
      // Development interop-server
      //ip_address: [138, 68, 250, 14, 8000],
      // Localhost interop-server
      ip_address: [138, 68, 250, 14, 8000, 'testuser', 'testpass'], // Set docker default
    };
    this.interopButtonPressed = this.interopButtonPressed.bind(this);
  }

  renderIPAddress() {
    return (
      <div className="ip_address">
        <form onSubmit={this.interopButtonPressed}>
          <div>
            <div className="ip_inputs">
              <input type="text" ref="quad_one"
                     placeholder={this.state.ip_address[0]}>
              </input>
              .
              <input type="text" ref="quad_two"
                     placeholder={this.state.ip_address[1]}>
              </input>
              .
              <input type="text" ref="quad_three"
                     placeholder={this.state.ip_address[2]}>
              </input>
              .
              <input type="text" ref="quad_four"
                     placeholder={this.state.ip_address[3]}>
              </input>
              :
              <input type="text" ref="port"
                     placeholder={this.state.ip_address[4]}>
              </input>
            </div>
            <div>
              <input type="text" ref="username" 
                     placeholder="testuser">
              </input>
            </div>
            <div>
              <input type="password" ref="password"
                     placeholder="testpass">
              </input>
            </div>
            <button className={`btn btn-sm align-middle btn-outline-dark ${
            !this.props.appState.interopBtnEnabled ? "disabled" : null}`}
                    id="interop_btn">
              {this.props.appState.interopBtnText}
            </button>
          </div>
        </form>
      </div>
    );
  }

  interopButtonPressed(e) {
    e.preventDefault();

    // Either IPv4 is 127.0.0.1:8000 or new values
    var quad_one = ( this.refs.quad_one.value === "" ) ?
                   this.state.ip_address[0] : this.refs.quad_one.value;
    var quad_two = ( this.refs.quad_two.value === "" ) ?
                   this.state.ip_address[1] : this.refs.quad_two.value;
    var quad_three = ( this.refs.quad_three.value === "" ) ?
                     this.state.ip_address[2] : this.refs.quad_three.value;
    var quad_four = ( this.refs.quad_four.value === "" ) ?
                    this.state.ip_address[3] : this.refs.quad_four.value;
    var port = ( this.refs.port.value === "" ) ?
                    this.state.ip_address[4] : this.refs.port.value;
    var username = ( this.refs.username.value === "" ) ?
                    'testuser' : this.refs.username.value;
    var password = ( this.refs.password.value === "" ) ?
                    'testpass' : this.refs.password.value;

    var new_ip_address = [ quad_one, quad_two, quad_three, quad_four, port, username, password ];
    this.setState({ip_address: new_ip_address});

    this.props.setAppState({ interopBtnText: "Connecting..." });
    this.props.setAppState({ interopBtnEnabled: false });
    this.props.socketEmit("connect_to_interop", new_ip_address);
  }

  renderUnitsSetting() {
    return (
      <div>
        <button className="btn btn-outline-dark"
          onClick={() => this.props.setAppState({metric: !this.props.appState.metric})}
        >
          {this.props.appState.metric ? "Metric" : "Imperial"}
        </button>
      </div>
    );
  }

  render() {
    return (
      <div className="Settings">
        <div className="text-center">
          <h1>Settings</h1>
        </div>
        <div className="settings_body">
          <div className="list-group">
            <div className="list-group-item">
              {this.renderIPAddress()}
            </div>
            <div className="list-group-item">
              {this.renderUnitsSetting()}
            </div>
          </div>
        </div>
      </div>
    );
  }
}




export default Settings;