import React from "react";

interface ReadoutProps {
  data: { key: string; values: (number | string)[] }[];
}

class Readout extends React.Component<ReadoutProps> {
  public render() {
    return (
      <div className="Readout">
        {this.props.data.map((comp, i) => (
          <KeyValue key={i} keyName={comp["key"]} values={comp["values"]} />
        ))}
      </div>
    );
  }
}

export default Readout;

interface KeyValueProps {
  key: number;
  keyName: string;
  values: (number | string)[];
}

const KeyValue = (props: KeyValueProps) => {
  let valueString = "";

  for (let i in props.values) {
    valueString += props.values[i];
  }

  return (
    <div className="KeyValue">
      <h1> {props.keyName} </h1>
      <h2> {valueString} </h2>
    </div>
  );
};
