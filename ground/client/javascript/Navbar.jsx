import React, { Component } from "react";

class Navbar extends Component {

  render() {
    return(
      <nav className="navbar navbar-expand-sm navbar-dark">
        <a className="navbar-brand" href="#">
          <img src="vector_logo.svg" height="45px" alt=""/>
        </a>
        <button className="navbar-toggler" type="button" data-toggle="collapse"
                data-target="#navbarNavAltMarkup" aria-controls="navbarNavAltMarkup"
                aria-expanded="false" aria-label="Toggle navigation">
          <span className="navbar-toggler-icon"></span>
        </button>
        <div className="collapse navbar-collapse" id="navbarNavAltMarkup">
          <div className="navbar-nav">
            <a className="nav-item nav-link active" href="#">Home <span className="sr-only">(current)</span></a>
            <a className="nav-item nav-link" href="#">Analytics</a>
          </div>
          {/* <button className="btn btn-sm align-middle btn-outline-light" id="interop_btn">Connect to Interop</button> */}
        </div>
      </nav>
    );
  }


}

export default Navbar;