import React, { Component } from 'react';

import './Tagging.css';

class Tagging extends Component {
  constructor(props) {
    super(props)
    this.state = {
        formValues: {},
        annValues: {shape: '',
                      shapeCol: '',
                      letter: '',
                      letterCol: '',
                      orient: ''
                    }
  };
    this.handleSubmit = this.handleSubmit.bind(this);
    this.handleChange = this.handleChange.bind(this);
}

handleChange(event) {
    event.preventDefault();
    let name = event.target.name;
    let value = event.target.value;
    let formValues = this.state.formValues;

    formValues[name] = value;

    this.setState((state)=>{
      return {
        formValues  //, curValues
      }
    });
}

handleSubmit(event) {
    event.preventDefault();
    let formValues = this.state.formValues;
    let annValues = this.state.annValues;

    for(var key in formValues){
      if(formValues[key] != ''){
        annValues[key] = formValues[key];
        formValues[key] = '';
      }
    }
    
    this.setState((state)=>{
      return{
        annValues,formValues
      }
    });
}

render(){
    return (
      <div class="Annotations">
      <h1>Annotations</h1>
        <form id = "Annotate" onSubmit={this.handleSubmit}>
          <div class="Shape">
          <label class ="dualField"> <span class="fieldVal">Shape: </span> <span class="curVal">{this.state.annValues["shape"]}</span>
            <br></br><input type="text" name="shape" placeholder="Shape" value={this.state.formValues["shape"]} onChange={this.handleChange}/>
          </label>
          <label class ="dualField"> <span class="fieldVal">Color: </span> <span class="curVal">{this.state.annValues["shapeCol"]}</span>
            <br></br><input type="text" name="shapeCol" placeholder="Shape Color" value={this.state.formValues["shapeCol"]} onChange={this.handleChange}/>
          </label>
          </div>

          <div class="Letter">
          <label class ="dualField"> <span class="fieldVal">Letter: </span> <span class="curVal">{this.state.annValues["letter"]}</span>
            <br></br><input type="text" name="letter" placeholder="Letter" value={this.state.formValues["letter"]} onChange={this.handleChange} />
          </label>
          <label class ="dualField"> <span class="fieldVal">Color: </span> <span class="curVal">{this.state.annValues["letterCol"]}</span>
            <br></br><input type="text" name="letterCol" placeholder="Letter Color" value={this.state.formValues["letterCol"]} onChange={this.handleChange} />
          </label>
          </div>

          <div class="Orient">
          <label class ="singField"> <span class="fieldVal">Orientation: </span> <span class="curVal">{this.state.annValues["orient"]}</span>
            <br></br><input type="text" name="orient" placeholder="Orientation" value={this.state.formValues["orient"]} onChange={this.handleChange} />
          </label>
          </div>

          <div class="saveButton">
          <input className="btn btn-primary" type="submit" value="Save" />
          </div>

        </form>
      </div>
  )
}
}

export default Tagging;