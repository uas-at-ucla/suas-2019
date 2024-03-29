import React from 'react';

import { Button, Modal, ModalHeader, ModalBody, ModalFooter } from 'reactstrap'

export default class ModalComponent extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            display: true
        };
        this.toggle = this.toggle.bind(this);
    }

    toggle() {
        this.setState({
            display: !this.state.display
        });
    }

    render() {
        var photoURL = "testPhotos/" + this.props.image;
        return (
            <div>
                <Modal isOpen={ this.state.display } size="lg">
                    <ModalHeader>Photoshop</ModalHeader>
                    <ModalBody>
                        <Button>Crop</Button>
                        <ModalFooter>
                            <img className="resize" src={photoURL}/>
                        </ModalFooter>
                        <Button color="danger" onClick={this.toggle}>Close</Button>
                    </ModalBody>
                </Modal>
            </div> 
        )
    }
}


