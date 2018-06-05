import React from 'react';
import io from "socket.io-client/dist/socket.io.js";

import { Button, Modal, ModalHeader, ModalBody, ModalFooter } from 'reactstrap'

export default class ModalComponent extends React.Component {
    constructor(props) {
        super(props);
        let img = new Image();
        img.src = props.image.src;
        this.state = {
            display: true,
            img_x_1: 0,
            img_y_1: 0,
            img_x_2: 0,
            img_y_2: 0,
            img_width: img.naturalWidth,
            img_height: img.naturalHeight
        };
        this.close = this.close.bind(this);
        this.send_manual = this.send_manual.bind(this);
        this.send_manual_test = this.send_manual_test.bind(this);
    }

    componentWillReceiveProps(nextProps) {
        if (nextProps.doubleClick !== this.props.doubleClick) {
            this.setState({display: true});
        }
    }

    componentWillMount() {
        this.vision_socket = io.connect("http://" + document.domain + ":8099");
        this.vision_socket.on("manual_request_done", data => {
            console.log(JSON.stringify(data));
        });
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

    send_manual_test() {
        this.send_manual(
          this.props.image.id,
          [this.state.img_x_1, this.state.img_y_1],
          [this.state.img_x_2, this.state.img_y_2]
        );
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
        this.props.socketEmit('cropped', img_id);
    }
    
    markCoords() {
        this.setState({
          img_x_1: this.state.img_x_2,
          img_y_1: this.state.img_y_2,
          img_x_2: Math.trunc(this.state.cur_img_x),
          img_y_2: Math.trunc(this.state.cur_img_y)
        });
    }

    render() {
        return (
            <div>
                <Modal isOpen={ this.state.display } size="lg">
                    <ModalHeader>Photoshop</ModalHeader>
                    <ModalBody>
                        <Button onClick={this.send_manual_test}>Crop</Button>
                        <ModalFooter>
                            <img className="resize" onClick={this.markCoords.bind(this)} onMouseMove={this._onMouseMove.bind(this)} src={this.props.image.src}/>
                        </ModalFooter>
                        <Button color="danger" onClick={this.close}>Close</Button>
                    </ModalBody>
                </Modal>
            </div> 
        )
    }
}


