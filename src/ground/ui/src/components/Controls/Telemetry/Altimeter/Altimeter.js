import React, { Component } from 'react';

import './Altimeter.css';

class Altimeter extends Component {
  constructor(props) {
    super(props)
    
    this.state = {
      percentage: 0
    }
    this.nextStep = this.nextStep.bind(this)
  }
  
  nextStep() {
    if (this.state.percentage === 100) return 
    this.setState({ percentage: this.state.percentage + 20 })
  }
  
  render() {
    return (
      <div className="Altimeter">        
        <div className="progress-bar">
          <div className={`filler ${this.state.percentage > 90 ? "warning" : "default"}`} style={{ height: `${this.state.percentage}%` }} />
        </div>

        <div style={{ marginTop: '20px' }}>  
          <button 
            onClick={this.nextStep}
           >
            ADD 
          </button>  
        </div>   
        
        <div style={{ marginTop: '20px' }}>
          <button1 
            onClick={() => this.setState({ percentage: 0 })}>
           
           Reset
          </button1>  
        </div>   
      </div>
    )
  }  
}

export default Altimeter;