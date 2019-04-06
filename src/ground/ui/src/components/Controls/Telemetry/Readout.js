import React from "react";

class Readout extends React.Component {
    render() {
        return (
            <div className="Readout">
                {this.props.data.map((comp , i) => <KeyValue key={i} keyName={comp["key"]} values={comp["values"]} />)}
            </div>
        );
    }
}

export default Readout;


const KeyValue = (props) => {
    let valueString = "";

    for(let i in props.values) {
        valueString += props.values[i];
    }

    return (
        <div className="KeyValue">
            <h1> {props.keyName} </h1>
            <h2> {valueString} </h2>
        </div>
    );
}
