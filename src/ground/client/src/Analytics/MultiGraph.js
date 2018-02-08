import React, { Component } from "react";

// Chart libraries
// https://github.com/jerairrest/react-chartjs-2
// https://www.youtube.com/watch?v=Ly-9VTXJlnA
import { Scatter } from "react-chartjs-2";

class MultiGraph extends Component {
  constructor(props) {
    super(props);
    let initDataSets = [];
    for (let i = 0; i < this.props.dataNames.length; i++) {
      initDataSets.push({
        label: this.props.dataNames[i],
        borderColor: this.props.colors[i],
        pointBorderColor: this.props.colors[i],
        data: []
      });
    }
    this.state = {
      graphData: {
        datasets: initDataSets
      }
    };
  }

  componentWillReceiveProps(nextProps) {
    if (nextProps.dataPoints) {
      this.setState((prevState, props) => {
        let newDataSets = [];
        for (let i = 0; i < nextProps.dataNames.length; i++) {
          newDataSets.push({
            label: this.props.dataNames[i],
            borderColor: this.props.colors[i],
            pointBorderColor: this.props.colors[i],
            data: [
              ...prevState.graphData.datasets[i].data,
              nextProps.dataPoints[i]
            ]
          });
        }
        return {
          graphData: {
            datasets: newDataSets
          }
        };
      });
    }
  }

  render() {
    return <Scatter data={this.state.graphData} />;
  }
}

export default MultiGraph;
