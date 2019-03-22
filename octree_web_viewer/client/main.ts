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
    private octreeIdControl: dat.GUIController;
    private gui: dat.GUI;
    private guiRenderControls: dat.GUI;
    public octreeId: string;  // octree identifier
    private renderArea: HTMLElement;

    private fetchDefaultOctreeId(): Promise<string> {
        const request = new Request(
            `/init_tree`,
            {
                method: 'GET',
                credentials: 'same-origin',
            }
        );

        const result = window
            .fetch(request)
            .then((response) => { return response.text(); }) // todo error handling?

        return result;
    }

    private initOctreeViewer(octreeId: string) {
        this.viewer = new OctreeViewer(this.scene, () => {
            this.needsRender = true;
        }, octreeId);
    }

    private addControls() {
        this.guiRenderControls = this.gui.addFolder('Render controls');
        this.guiRenderControls
            .add(this.viewer.material.uniforms['size'], 'value')
            .name('Point size')
            .onChange(() => {
                this.needsRender = true;
            });
        this.guiRenderControls
            .add(this.viewer.material.uniforms['alpha'], 'value', 0, 1)
            .name('Transparency')
            .onChange(() => {
                this.viewer.alphaChanged();
                this.needsRender = true;
            });
        this.guiRenderControls
            .add(this.viewer.material.uniforms['gamma'], 'value')
            .name('Gamma')
            .onChange(() => {
                this.needsRender = true;
            });
        this.guiRenderControls
            .add(this.viewer, 'maxLevelToDisplay', 0, 7)
            .name('Moving details')
            .step(1)
            .onChange(() => {
                this.needsRender = true;
            });
    }

    private getViewPortSize(): [number, number] {
        let width = this.renderArea.clientWidth;
        let height = this.renderArea.clientHeight;
        return [width, height];
    }
    private initCamera() {
        const VIEW_ANGLE = 45;
        const NEAR = 0.1;
        const FAR = 10000;
        let [width, height] = this.getViewPortSize();
        this.camera = new THREE.PerspectiveCamera(
            VIEW_ANGLE,
            width / height,
            NEAR,
            FAR
        );
        this.camera.position.z = 150;
        this.camera.updateMatrix();
        this.camera.updateMatrixWorld(false);
    }

    private initScene() {
        this.scene = new THREE.Scene();
        this.scene.add(this.camera);
    }

    private initRenderer() {
        this.renderer = new THREE.WebGLRenderer();

        let [width, height] = this.getViewPortSize();
        this.renderer.setSize(width, height);
        this.renderArea.appendChild(this.renderer.domElement);
        this.controller = new FirstPersonController(
            this.camera,
            this.renderer.domElement
        );
        this.needsRender = true;
        this.lastFrustumUpdateTime = 0;
        this.lastMoveTime = 0;
    }

    private cleanup() {
        // TODO(negin-z) block requests from the viewer that is going to be replaced
        this.removeControls();
        if (this.renderer) {
            this.renderArea.removeChild(this.renderer.domElement);
            this.renderer.dispose();
        }
    }

    private removeControls() {
        if (this.guiRenderControls) {
            this.gui.removeFolder(this.guiRenderControls);
        }
    }

    private resetOctree() {
        this.cleanup();
        this.initCamera();
        this.initScene();
        this.initRenderer();
        this.initOctreeViewer(this.octreeId);
        this.addControls();
    }

    private setOctreeId = (newOctreeId: string) => {
        this.octreeIdControl.setValue(newOctreeId);
    }

    private run = () => {
        this.resetOctree();
        this.animate();
    }

    public init() {
        this.renderArea = document.getElementById('renderArea');
        this.octreeId = "loading...";
        this.gui = new GUI();

        this.octreeIdControl =
            this.gui
                .add(this, 'octreeId')
                .name('Point Cloud ID')
                .onFinishChange(this.run);

        // TODO(negin-z) error handling
        this.fetchDefaultOctreeId()
            .then(this.setOctreeId)
            .then(this.run);

        window.addEventListener('resize', () => this.onWindowResize(), false);
    }

    private onWindowResize() {
        // TODO(cvitadello) why is the window size used here?
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
        if (this.lastFrustumUpdateTime <= this.lastMoveTime &&
            time - this.lastFrustumUpdateTime > 250) {
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
            );
        }

        if (this.needsRender) {
            this.needsRender = false;
            // TODO(hrapp): delete invisible nodes and free memory again.
            this.renderer.render(this.scene, this.camera);
        }
    }
}

let app = new App();
app.init();
