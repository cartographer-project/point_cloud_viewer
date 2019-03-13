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
import { GUI } from 'dat.gui';
import { FirstPersonController } from './control';
import { OctreeViewer } from './octree_viewer';

class App {
  private camera: THREE.PerspectiveCamera;
  private scene: THREE.Scene;
  private controller: FirstPersonController;
  private viewer: OctreeViewer;
  private renderer: THREE.WebGLRenderer;
  private lastFrustumUpdateTime: number;
  private lastMoveTime: number;
  private needsRender: boolean;
  private forceClear: boolean;

  public run() {
    let renderArea = document.getElementById('renderArea');
    this.renderer = new THREE.WebGLRenderer();
    this.renderer.setSize(renderArea.clientWidth, renderArea.clientHeight);
    renderArea.appendChild(this.renderer.domElement);

    const VIEW_ANGLE = 45;
    const NEAR = 0.1;
    const FAR = 10000;
    this.camera = new THREE.PerspectiveCamera(
      VIEW_ANGLE,
      renderArea.clientWidth / renderArea.clientHeight,
      NEAR,
      FAR
    );
    this.camera.position.z = 150;
    this.camera.updateMatrix();
    this.camera.updateMatrixWorld(false);

    this.scene = new THREE.Scene();
    this.scene.add(this.camera);

    this.controller = new FirstPersonController(
      this.camera,
      this.renderer.domElement
    );

    this.lastFrustumUpdateTime = 0;
    this.lastMoveTime = 0;
    this.needsRender = true;
    this.forceClear = false;
    this.viewer = new OctreeViewer(this.scene, () => {
      this.needsRender = true;
    });
    const gui = new GUI();
    gui
      .add(this.viewer.material.uniforms['size'], 'value')
      .name('Point size')
      .onChange(() => {
        this.needsRender = true;
      });
    gui
      .add(this.viewer.material.uniforms['alpha'], 'value', 0, 1)
      .name('Transparency')
      .onChange(() => {
        this.viewer.alphaChanged();
        this.needsRender = true;
      });
    gui
      .add(this.viewer.material.uniforms['gamma'], 'value')
      .name('Gamma')
      .onChange(() => {
        this.needsRender = true;
      });
    gui
      .add(this.viewer, 'maxLevelToDisplay', 0, 7)
      .name('Moving details')
      .step(1)
      .onChange(() => {
        this.needsRender = true;
      });
    gui
      .add(this.viewer, 'uuid')
      .name('Point Cloud ID')
      .onFinishChange(() => {
        this.forceClear = true;
        this.viewer.load_new_tree();
        this.needsRender = true;
      });

    window.addEventListener('resize', () => this.onWindowResize(), false);
    this.animate();
  }

  private onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    // Force a reload of the visible nodes.
    this.lastFrustumUpdateTime = 0;
  }

  public animate() {
    requestAnimationFrame(() => this.animate());

    const time = performance.now();
    if (this.controller.update()) {
      this.lastMoveTime = time;
      this.viewer.setMoving(true);
      this.needsRender = true;
    }
    if (time - this.lastMoveTime > 250) {
      this.viewer.setMoving(false);
      this.needsRender = true;
    }
    if (this.forceClear ||
      (this.lastFrustumUpdateTime <= this.lastMoveTime &&
        time - this.lastFrustumUpdateTime > 250)
    ) {
      this.camera.updateMatrixWorld(false);
      this.lastFrustumUpdateTime = time;
      const matrix = new THREE.Matrix4().multiplyMatrices(
        this.camera.projectionMatrix,
        this.camera.matrixWorldInverse
      );
      this.viewer.frustumChanged(
        matrix,
        this.renderer.context.canvas.width,
        this.renderer.context.canvas.height,
        this.viewer.uuid
      );
    }

    if (this.needsRender) {
      this.needsRender = false;
      // TODO(hrapp): delete invisible nodes and free memory again.
      this.renderer.render(this.scene, this.camera, undefined, this.forceClear);
      this.forceClear = false;
    }
  }
}

let app = new App();
app.run();
