import React, { Component } from 'react';
import { connect } from 'react-redux';
import {
  Container,
  Row,
  Col
} from 'reactstrap';
import './Vision.css';
import Map from './Map/Map';
import Tagging from './Tagging/Tagging';
import Pipeline from './Pipeline/Pipeline';

const mapStateToProps = state => {return {vision: state.vision};};
const mapDispatchToProps = state => {};

class Vision extends Component {
  render() {
    return (
      <div className="Vision">
        <div className="map-overlay">
        <Tagging />
        </div>
        <Map />
      </div>
    );
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(Vision);
