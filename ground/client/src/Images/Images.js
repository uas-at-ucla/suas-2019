import React, { Component } from 'react';

class Images extends Component {

  render() {
    let image = this.props.appState.testImage ? <img src={`data:image/jpg;base64,${this.props.appState.testImage.base64}`}/> : null
    return (
      <div className="Images">
        <h1>Render images here</h1>
        <button onClick={this.getTestImage}>Test!</button>
        {image}
      </div>
    );
  }

  getTestImage = () => {
    this.props.socketEmit('test_image', {fileName: 'send_to_ground_test.jpg'});
  }
}

export default Images;
