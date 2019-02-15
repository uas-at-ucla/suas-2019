import React, { Component } from 'react';
import * as THREE from 'three';

import NavballImg from './navball.png';


class Navball extends Component {
    constructor(props) {
        super(props);

        // State
    }


    componentDidMount() {
        const WIDTH = 200;
        const HEIGHT = WIDTH;

        this.scene = new THREE.Scene();

        this.camera = new THREE.PerspectiveCamera(41 , 1 , 1 , 2000);
        this.scene.add(this.camera);
        this.camera.position.set(0 , 0 , 120);
        this.camera.lookAt(this.scene.position);


        this.renderer = new THREE.WebGLRenderer({antialias:true});
        this.renderer.setSize(WIDTH, HEIGHT);
        document.getElementById("navball").appendChild(this.renderer.domElement);

        var light = new THREE.AmbientLight( 0xffffff , 1.5 );;
        this.scene.add(light);


        var sphereGeom = new THREE.SphereGeometry(40 , 48 , 32);
        var texture = new THREE.TextureLoader().load(NavballImg);
        var material = new THREE.MeshPhongMaterial({map: texture});
        this.ball = new THREE.Mesh(sphereGeom , material);
        this.ball.position.set(0 , -3.1415926 / 2 , 0);
        this.scene.add(this.ball);

        this.start();
    }


    start = () => {
        if (!this.frameId) {
          this.frameId = requestAnimationFrame(this.animate)
        }
    }


    stop = () => {
        cancelAnimationFrame(this.frameId)
    }


    animate = () => {
       this.ball.rotation.x = this.props.data["navX"];
       this.ball.rotation.y = this.props.data["navY"] - 3.1415926 / 2;     // Zero me!
       this.ball.rotation.z = this.props.data["navZ"];
       this.renderScene()
       this.frameId = window.requestAnimationFrame(this.animate)
    }


    renderScene = () => {
      this.renderer.render(this.scene, this.camera)
    }



    render() {
        return(
            <div className="AttitudeIndicator" id="navball" ref={(mount) => {this.mount = mount}}>
            </div>
        );
    }
}


export default Navball;