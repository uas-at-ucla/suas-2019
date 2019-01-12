import React from 'react';

import Logo from "../../../graphics/uasLogo.svg";

class UasLogo extends React.Component {
    render() {
        return (
            <div className="component-background">
                <img className="logo-img" src={Logo}/>
            </div>
        );
    }
}

export default UasLogo;