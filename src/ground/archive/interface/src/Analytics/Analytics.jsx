import React, { Component } from "react";
import Graph from "./Graph";
import Checkbox from "./Checkbox";
import "./Analytics.css";
import MultiGraph from "./MultiGraph";


class Analytics extends Component {
  constructor(props) {
    super(props);

    this.updateGraphVis = this.updateGraphVis.bind(this);

    this.multigraphs = [
      {
        graphTitle: "Velocity",
        dataNames: ["X Velocity", "Y Velocity", "Z Velocity"],
        colors: ["#f00", "#0f0", "#00f"],
        dataPoints: [
          {
            xData: "flight_time",
            yData: "velocity_x"
          },
          {
            xData: "flight_time",
            yData: "velocity_y"
          },
          {
            xData: "flight_time",
            yData: "velocity_z"
          }
        ]
      }
    ];

    this.singlegraphs = [
      {
        dataName: "X Velocity",
        color: "#f00",
        xData: "flight_time",
        yData: "velocity_x"
      },
      {
        dataName: "Y Velocity",
        color: "#0f0",
        xData: "flight_time",
        yData: "velocity_y"
      },
      {
        dataName: "Z Velocity",
        color: "#00f",
        xData: "flight_time",
        yData: "velocity_z"
      },
      {
        dataName: "Battery Voltage",
        color: "rgba(255,140,0,1)",
        xData: "flight_time",
        yData: "battery_voltage"
      }
    ];

    let graphVis = {};
    for (let obj in this.multigraphs) {
      graphVis[obj.graphTitle] = "none";
    }

    for (let obj in this.singlegraphs) {
      graphVis[obj.dataName] = "none";
    }

    this.state = {
      showGraphs: graphVis
    };
  }

  updateGraphVis(graphName, visibility) {
    console.log(graphName + ": " + visibility);
    if (visibility) {
      this.setState(prevState => {
        let newState = {
          showGraphs: {
            ...prevState.showGraphs
          }
        };
        newState.showGraphs[graphName] = "block";
        return newState;
      });
    } else {
      this.setState(prevState => {
        let newState = {
          showGraphs: {
            ...prevState.showGraphs
          }
        };
        newState.showGraphs[graphName] = "none";
        return newState;
      });
    }
  }

  // Render charts base on checkboxes
  // The only props we want to pass into Graph component are
  // X and Y data points - this makes the component reuseable
  renderGraphs() {
    if (this.telemetry) {
      return (
        <div>
          {/* Multi-dataset Graphs */
          this.multigraphs.map(objMapped => {
            let dataPoints = [];
            for (let dataPoint of objMapped.dataPoints) {
              dataPoints.push({
                x: this.telemetry['status'][dataPoint.xData],
                y: this.telemetry['sensors'][dataPoint.yData]
              });
            }
            return (
              <div
                style={{
                  display: this.state.showGraphs[objMapped.graphTitle]
                }}
              >
                <MultiGraph
                  dataNames={objMapped.dataNames}
                  colors={objMapped.colors}
                  dataPoints={dataPoints}
                />
              </div>
            );
          })}

          {/* Single Graphs */
          this.singlegraphs.map(objMapped => {
            return (
              <div
                style={{
                  display: this.state.showGraphs[objMapped.dataName]
                }}
              >
                <Graph
                  dataName={objMapped.dataName}
                  color={objMapped.color}
                  dataPoint={{
                    x: this.telemetry['status'][objMapped.xData],
                    y: this.telemetry['sensors'][objMapped.yData]
                  }}
                />
              </div>
            );
          })}
        </div>
      );
    }
  }

  render() {
    this.telemetry = this.props.appState.telemetry;

    return (
      <div className="Analytics col-md-offset-1 col-md-10">
        <h1>Analytics</h1>

        <table class="text-center" width="80%">
          <tr>
            {this.multigraphs.map(objMapped => {
              return <th>{objMapped.graphTitle}</th>;
            })}
            {this.singlegraphs.map(objMapped => {
              return <th>{objMapped.dataName}</th>;
            })}
          </tr>
          <tr>
            {this.multigraphs.map(objMapped => {
              return (
                <td>
                  <Checkbox
                    valName={objMapped.graphTitle}
                    updateVal={this.updateGraphVis}
                  />
                </td>
              );
            })}
            {this.singlegraphs.map(objMapped => {
              return (
                <td>
                  <Checkbox
                    valName={objMapped.dataName}
                    updateVal={this.updateGraphVis}
                  />
                </td>
              );
            })}
          </tr>
        </table>

        {this.renderGraphs()}
      </div>
    );
  }
}

export default Analytics;
