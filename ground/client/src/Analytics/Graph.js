import React, { Component } from 'react';

// Chart libraries
// https://github.com/jerairrest/react-chartjs-2
// https://www.youtube.com/watch?v=Ly-9VTXJlnA
import {Bar, Line, Pie, Scatter} from 'react-chartjs-2';

class Graph extends Component {

  constructor(props){
	super(props);
	this.state = {
		graphData: {
			datasets:
			[
				{
				label: props.dataName,
				borderColor: props.color,
				pointBorderColor: props.color,
				data: []
				}
			]
		}
	};
  }

  componentWillReceiveProps(nextProps)
  {
	if (nextProps.dataPoint)
	{
		this.setState((prevState, props) =>
		{
			return {
				graphData: {
					datasets:
					[
						{
						label: props.dataName,
						borderColor: props.color,
						pointBorderColor: props.color,
						data: [...prevState.graphData.datasets[0].data, nextProps.dataPoint]
						}
					]
				}
			};
		});
	}
  }

  render() {

	  let options =
	  {
			scales: {
				xAxes: [{
					type: 'linear',
					position: 'bottom'
				}]
			}
	  };
	  /*
	  const data = (canvas) => {
		  const ctx = canvas.getContext("2d")
		  const gradient = ctx.createLinearGradient(0,0,100,0);
		  return {
			   backgroundColor: gradient
		  }
	  }
	  */

	  return (
		  	<Scatter data={this.state.graphData} />
	  );
  }
}

export default Graph;