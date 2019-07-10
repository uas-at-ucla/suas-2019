import React, { Component } from "react";
import "./AttitudeIndicator.css";
import NavballImg from "./navball.png";
import PositionRef from "./navball_pos_ref.svg";
import * as THREE from "three";

/*
More navball skins:
https://forum.kerbalspaceprogram.com/index.php?/topic/164158-13-navballtexturechanger-v16-8717/&do=findComment&comment=3142105
*/

const sphereRadius = 1;
const cameraDistance = 8;
const fov = ((Math.asin(sphereRadius / cameraDistance) * 180) / Math.PI) * 2;

const MARGIN = 5; // from CSS
const WIDTH = 200 - MARGIN * 2;
const HEIGHT = WIDTH;

interface Props {
  data: {
    roll: number;
    pitch: number;
    yaw: number;
  };
}

class Navball extends Component<Props> {
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(
    fov,
    1,
    cameraDistance - sphereRadius,
    cameraDistance + sphereRadius
  );
  private renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  private ballContainer = new THREE.Object3D();
  private frameId = NaN;

  private navballDidMount = (navball: HTMLElement | null) => {
    this.scene.add(this.camera);
    this.camera.position.set(cameraDistance, 0, 0);
    this.camera.up.set(0, 0, 1);
    this.camera.lookAt(0, 0, 0);

    this.renderer.setSize(WIDTH, HEIGHT);
    navball && navball.appendChild(this.renderer.domElement);

    var light = new THREE.AmbientLight(0xffffff, 1.5);
    this.scene.add(light);

    var sphereGeom = new THREE.SphereGeometry(sphereRadius, 48, 32);
    var texture = new THREE.TextureLoader().load(NavballImg);
    texture.offset.x = 1 / 4; // 0 radians yaw is East
    texture.wrapS = THREE.RepeatWrapping;
    var material = new THREE.MeshPhongMaterial({ map: texture });
    var ball = new THREE.Mesh(sphereGeom, material);
    ball.position.set(0, 0, 0);
    ball.rotation.x = Math.PI / 2;
    this.ballContainer.add(ball);
    this.ballContainer.position.copy(ball.position);
    this.scene.add(this.ballContainer);

    this.updateRotation();

    this.frameId = requestAnimationFrame(this.animate);
  };

  private updateRotation = () => {
    this.ballContainer.rotation.x = this.props.data["roll"];
    this.ballContainer.rotation.y = -this.props.data["pitch"];
    this.ballContainer.rotation.z = this.props.data["yaw"];
  };

  public componentWillUnmount() {
    cancelAnimationFrame(this.frameId);
  }

  private animate = () => {
    this.frameId = window.requestAnimationFrame(this.animate);
    this.renderer.render(this.scene, this.camera);
  };

  public componentDidUpdate(prevProps: Props) {
    this.updateRotation();
  }

  public render() {
    return (
      <div className="AttitudeIndicator">
        <div className="navball-background"></div>
        <div className="navball" ref={this.navballDidMount}></div>
        <div className="position-ref-container">
          <img className="position-ref" src={PositionRef} alt="" />
        </div>
      </div>
    );
  }
}

export default Navball;
