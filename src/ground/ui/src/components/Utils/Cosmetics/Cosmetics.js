import React from 'react';

import "./Cosmetics.css";
import UasLogo from './UasLogo';

class Cosmetics extends React.Component {
    render() {
        return(
            <UasLogo onClick={this.props.onClick}/>
        );
    }
}

export default Cosmetics;