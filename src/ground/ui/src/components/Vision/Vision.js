import React, { Component } from 'react';
import { Container, Row, Col } from 'reactstrap';


import './Vision.css';
import Map from './Map/Map';
import Tagging from './Tagging/Tagging';
import Pipeline from './Pipeline/Pipeline';

class Vision extends Component {
  render() {
    return (
      <div className="Vision">
        <Container>
          <Row>

            <div className="col-md-4">
              <Map/>
            </div>

            <div className="col-md-4">
              <Tagging/>
            </div>

            <div className="col-md-4">
              <Pipeline/>
            </div>

          </Row>
        </Container>
      </div>
    );
  }
}

export default Vision;