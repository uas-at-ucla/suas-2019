import React from 'react';

import { Button, Modal, ModalHeader, ModalBody, ModalFooter } from 'reactstrap'

const object_keys = [
    "type",
    "latitude",
    "longitude",
    "orientation",
    "shape",
    "background_color",
    "alphanumeric",
    "alphanumeric_color"
]

export default class Classify extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            display: true,
            "type": "standard",
            "latitude": 0,
            "longitude": 0,
            "orientation": "",
            "shape": "",
            "background_color": "",
            "alphanumeric": "",
            "alphanumeric_color": ""
        };
        if (props.object) {
            for (let key of object_keys) {
                this.state[key] = props.object[key];
            }
            this.state.submitted = true;
        }
        this.close = this.close.bind(this);
        this.submit_to_interop = this.submit_to_interop.bind(this);
    }

    componentWillReceiveProps(nextProps) {
        if (nextProps.doubleClick !== this.props.doubleClick) {
            let newState = {display: true};
            if (nextProps.object) {
                for (let key of object_keys) {
                    newState[key] = nextProps.object[key];
                }
                newState.submitted = true;
            } else {
                newState.submitted = false;
            }
            this.setState(newState);
        }
    }

    _onMouseMove(e) {
        let bounds = e.target.getBoundingClientRect();
        this.setState({
          cur_img_x:
            (e.clientX - bounds.left) /
            (bounds.right - bounds.left) *
            this.state.img_width,
          cur_img_y:
            (e.clientY - bounds.top) /
            (bounds.bottom - bounds.top) *
            this.state.img_height
        });
    }

    close() {
        this.setState({display: false});
    }

    submit_to_interop() {
        this.setState({submitted: true});
        let object = {}
        for (let key of object_keys) {
            object[key] = this.state[key];
        }
        this.props.socketEmit("classified", {
            id: this.props.image.id,
            object: object
        });
    }

    render() {
        return (
            <div>
                <Modal isOpen={ this.state.display } size="lg">
                    <ModalHeader>Photoshop</ModalHeader>
                    <ModalBody>
                        <Button onClick={this.submit_to_interop} disabled={this.state.submitted}>Submit</Button>
                        <ModalFooter>
                            <img className="resize" src={this.props.image.src}/>
                            <div>
                                {object_keys.map((key, index) => 
                                    <div key={index}>
                                        {key}:<input disabled={this.state.submitted} value={this.state[key]} onChange={e => {let newState = {}; newState[key] = e.target.value; this.setState(newState);}}></input>
                                    </div>
                                )}
                            </div>
                        </ModalFooter>
                        <Button color="danger" onClick={this.close}>Close</Button>
                    </ModalBody>
                </Modal>
            </div> 
        )
    }
}
