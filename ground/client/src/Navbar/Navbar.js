import React from 'react';
import './Navbar.css'
import logo from '../images/vector_logo.svg';

const navbar = (props) => {
  return (
    <div className="Navbar">
      <nav className="navbar navbar-expand-sm navbar-dark">
        <a className="navbar-brand" href="/" style={{padding: 0}}>
          <img src={logo} height="45px" alt=""/>
        </a>
        <button className="navbar-toggler" type="button" data-toggle="collapse"
          data-target="#nav_container" aria-controls="nav_container"
          aria-expanded="false" aria-label="Toggle navigation">
          <span className="navbar-toggler-icon"></span>
        </button>
        <div className="collapse navbar-collapse" id="nav_container">
          <div className="navbar-nav">
            <a className="nav-item nav-link active" href="#">Home <span className="sr-only">(current)</span></a>
            <a className="nav-item nav-link" href="#">Analytics</a>
          </div>
          <button className={`btn btn-sm align-middle btn-outline-light ${!props.interopBtnEnabled ? 'disabled' : null}`}
            id="interop_btn" onClick={props.interopBtnClick}>{props.interopBtnText}</button>
        </div>
      </nav>
    </div>
  )
};

export default navbar