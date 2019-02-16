import React from "react";


class KeyValue extends React.Component {
    render() {
        let valueString = "";

        for(let i in this.props.values) {
            valueString += this.props.values[i];
        }

        return (
            <div className="KeyValue">
                <h1> {this.props.keyName} </h1>
                <h2> {valueString} </h2>
            </div>
        );
    }
}

export default KeyValue;