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
            for (let key in props.object) {
                this.state[key] = props.object[key];
            }
        }
        this.state.submitted = props.submitted;
        this.close = this.close.bind(this);
        this.submit_to_interop = this.submit_to_interop.bind(this);
    }

    componentWillReceiveProps(nextProps) {
        let newState = {};
        if (nextProps.doubleClick !== this.props.doubleClick) {
            newState.display = true;
            newState.submitted = nextProps.submitted;
        }
        if (nextProps.object !== this.props.object) {
            for (let key in nextProps.object) {
                newState[key] = nextProps.object[key];
            }
        }
        this.setState(newState);
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
            if (key === "latitude" || key === "longitude") {
                object[key] = Number(object[key]);
            }
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
                        {this.props.appState.manualClassifiedImages.find(el => el.id === this.props.image.id) ? <span>Submitted!</span> : null}
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
