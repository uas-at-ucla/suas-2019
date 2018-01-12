import React, { Component } from 'react';

// Chart libraries
// https://github.com/jerairrest/react-chartjs-2
// https://www.youtube.com/watch?v=Ly-9VTXJlnA
import { Line } from 'react-chartjs-2';

class Graph extends Component {

  render() {
	  const data = (canvas) => {
		  const ctx = canvas.getContext("2d")
		  const gradient = ctx.createLinearGradient(0,0,100,0);
		  return {
			   backgroundColor: gradient
		  }
	  }

	  return (
		  <Line data={data} />
	  )
  }
}

export default Graph;