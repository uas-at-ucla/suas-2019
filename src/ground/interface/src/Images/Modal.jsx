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
        this.sendCropRequest = this.sendCropRequest.bind(this);
        this.cropTarget = this.cropTarget.bind(this);
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

    componentDidMount() {
        const loaded_img = this.refs.loaded_img;
        const img_canvas = this.refs.img_canvas;
        img_canvas.width = loaded_img.naturalWidth;
        img_canvas.height = loaded_img.naturalHeight;
        const context = img_canvas.getContext("2d");

        loaded_img.onload = () => {
            context.drawImage(loaded_img, 0, 0);
            context.strokeStyle = "#ff0000";
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

    cropTarget() {
        this.sendCropRequest(
          this.props.image.id,
          [this.state.img_x_1, this.state.img_y_1],
          [this.state.img_x_2, this.state.img_y_2]
        );
    }

    sendCropRequest(img_id, top_left, bottom_right) {
        let json = {
            'event_name': 'snip',
            'args': {
                'img_id': img_id,
                'yolo_results': [{
                    'topleft':      {'x': top_left[0], 'y': top_left[1]},
                    'bottomright':  {'x': bottom_right[0], 'y': bottom_right[1]}
                }]
            }
        }
        this.vision_socket.emit("manual_request", json);
        this.props.socketEmit('cropped', img_id);
        console.log(json)
    }

    clearSelection() {
        this.setState({
          img_x_1: null,
          img_y_1: null,
          img_x_2: null,
          img_y_2: null
        })

        const canvas = this.refs.img_canvas;
        const context = canvas.getContext("2d");
        context.drawImage(this.refs.loaded_img, 0, 0);
    }
    
    markCoords() {
        const new_x = Math.trunc(this.state.cur_img_x);
        const new_y = Math.trunc(this.state.cur_img_y);
        const canvas = this.refs.img_canvas;
        const context = canvas.getContext("2d");


        // No first top left coords -> just mark the top left corner
        if (this.state.img_x_1 === null && this.state.img_y_1 === null) {
            this.setState({
              img_x_1: new_x,
              img_y_1: new_y,
            });

            context.beginPath();
            context.arc(new_x, new_y, 5, 0, 2*Math.PI);
            context.stroke();
        }
        // no bottom right coords -> mark, and then draw the rectangle
        else if (this.state.img_x_2 === null && this.state.img_y_2 === null) {
            this.setState({
              img_x_2: new_x,
              img_y_2: new_y
            });

            context.beginPath();
            context.arc(new_x, new_y, 5, 0, 2*Math.PI);
            context.stroke();

            context.strokeRect(this.state.img_x_1, this.state.img_y_1, new_x-this.state.img_x_1, new_y-this.state.img_y_1);
        }
        // all coords already selected -> clear selection
        else {
            this.clearSelection();
        }
    }

    render() {
        return (
            <div>
                <Modal isOpen={ this.state.display } size="lg">
                    <ModalHeader>Photoshop</ModalHeader>
                    <ModalBody>
                        <Button onClick={this.cropTarget}>Crop</Button>
                        <ModalFooter>
                            <canvas ref="img_canvas" className="resize" onClick={this.markCoords.bind(this)} onMouseMove={this._onMouseMove.bind(this)} onContextMenu={(e) => {e.preventDefault();this.clearSelection()}}/>
                            <img ref="loaded_img" style={{display: 'none'}} src={this.props.image.src}/>
                        </ModalFooter>
                        <Button color="danger" onClick={this.close}>Close</Button>
                    </ModalBody>
                </Modal>
            </div> 
        )
    }
}


