import React, { Component } from 'react';

// Chart libraries
// https://github.com/jerairrest/react-chartjs-2
// https://www.youtube.com/watch?v=Ly-9VTXJlnA
import { Scatter } from 'react-chartjs-2';

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
	  if (this.props.options)
		  return (<Scatter data={Object.assign({}, this.state.graphData, this.props.options)}/>);
	  else
	      return (<Scatter data={this.state.graphData} />);
  }
}

export default Graph;
