import React, { Component } from "react";

// Chart libraries
// https://github.com/jerairrest/react-chartjs-2
// https://www.youtube.com/watch?v=Ly-9VTXJlnA
import { Scatter } from "react-chartjs-2";

class Graph extends Component {
  constructor(props) {
    super(props);
    this.state = {
      graphData: {
        datasets: [
          {
            label: props.dataName,
            borderColor: props.color,
            pointBorderColor: props.color,
            data: []
          }
        ]
      },
    };

    this.skipRenders = 0;
  }

  shouldComponentUpdate(nextProps, nextState) {
    if(this.skipRenders % 5 == 0) {
      this.skipRenders += 1;
      return true;
    }

    this.skipRenders += 1;
    return false;
  }

  componentWillReceiveProps(nextProps) {
    if (nextProps.dataPoint) {
      this.setState((prevState, props) => {
        let data = [
          ...prevState.graphData.datasets[0].data,
          nextProps.dataPoint
        ];

        // Delete old points.
        let history = 1000
        if(data.length > history) {
          data.splice(0, history - 100);
        }

        return {
          graphData: {
            datasets: [{
                pointRadius: 1,
                label: props.dataName,
                borderColor: props.color,
                pointBorderColor: props.color,
                data: data,
            }]
          }
        };
      });
    }
  }

  render() {
    if (this.props.options) {
      let height = 60;
      return (
        <Scatter
          data={this.state.graphData}
          options={this.props.options}
          height={height}
        />
      );
    } else {
      return <Scatter data={this.state.graphData} />;
    }
  }
}

export default Graph;
