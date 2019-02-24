import React from "react";

import KeyValue from "./KeyValue";


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