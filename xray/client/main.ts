// Copyright 2016 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

import * as THREE from 'three';

import {Maps2DController} from './control';
import {XRayViewer} from './xray_viewer';

class App {
  private camera: THREE.OrthographicCamera;
  private scene: THREE.Scene;
  private controller: Maps2DController;
  private viewHasChanged: boolean;
  private viewer: XRayViewer;
  private renderer: THREE.WebGLRenderer;

  public run() {
    let renderArea = document.getElementById('renderArea');
    this.renderer = new THREE.WebGLRenderer();
    this.renderer.setSize(renderArea.clientWidth, renderArea.clientHeight);
    this.renderer.setClearColor(0xffffff);
    renderArea.appendChild(this.renderer.domElement);

    this.camera = new THREE.OrthographicCamera(
      renderArea.clientWidth / -2,
      renderArea.clientWidth / 2,
      renderArea.clientHeight / 2,
      renderArea.clientHeight / -2,
      -100,
      100
    );

    this.viewHasChanged = true;
    this.camera.updateMatrix();
    this.camera.updateMatrixWorld(false);
    this.scene = new THREE.Scene();
    this.viewer = new XRayViewer(this.scene, '');
    this.scene.add(this.camera);

    this.controller = new Maps2DController(
      this.camera,
      this.renderer.domElement
    );

    window.addEventListener('resize', () => this.onWindowResize(), false);
    this.animate();
  }

  private onWindowResize() {
    this.camera.left = -window.innerWidth / 2;
    this.camera.right = window.innerWidth / 2;
    this.camera.top = window.innerHeight / 2;
    this.camera.bottom = -window.innerHeight / 2;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  public animate() {
    requestAnimationFrame(() => this.animate());

    this.viewHasChanged = this.controller.update() || this.viewHasChanged;
    if (this.viewHasChanged && this.viewer.isInitialized()) {
      this.viewHasChanged = false;
      const matrix = new THREE.Matrix4().multiplyMatrices(
        this.camera.projectionMatrix,
        this.camera.matrixWorldInverse
      );
      // The camera's zoom is exactly the number of pixels per meter.
      this.viewer.frustumChanged(matrix, this.camera.zoom);
    }
    this.renderer.render(this.scene, this.camera);
  }
}

let app = new App();
app.run();
