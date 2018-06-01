import React from 'react';
import io from "socket.io-client/dist/socket.io.js";

import { Button, Modal, ModalHeader, ModalBody, ModalFooter } from 'reactstrap'

export default class ModalComponent extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            display: true
        };
        this.toggle = this.toggle.bind(this);
        this.send_manual = this.send_manual.bind(this);
        this.send_manual_test = this.send_manual_test.bind(this);
    }

    componentWillMount() {
        this.vision_socket = io.connect("http://" + document.domain + ":8099");
        this.vision_socket.on("manual_request_done", data => {
            console.log(JSON.stringify(data));
        });
    }

    toggle() {
        this.setState({
            display: !this.state.display
        });
    }

    send_manual_test() {
        this.send_manual("4fg2E94PTjKwb681diPaGA", [25, 25], [50, 50]);
    }

    send_manual(img_id, top_left, bottom_right) {
        this.vision_socket.emit("manual_request", {
            'event_name': 'snip',
            'args': {
                'img_id': img_id,
                'yolo_results': [{
                    'topleft':      {'x': top_left[0], 'y': top_left[1]},
                    'bottomright':  {'x': bottom_right[0], 'y': bottom_right[1]}
                }]
            }
        });
    }

    render() {
        var photoURL = "testPhotos/" + this.props.image;
        return (
            <div>
                <Modal isOpen={ this.state.display } size="lg">
                    <ModalHeader>Photoshop</ModalHeader>
                    <ModalBody>
                        <Button onClick={this.send_manual_test}>Crop</Button>
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


