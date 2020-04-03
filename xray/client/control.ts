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

enum MouseState {
  NONE,
  DRAG,
}

// A Google maps like controller interface: Click & drag to move, mouse wheel
// to zoom.
// TODO(sirver): This should be removed and OrbitControl used.
export class Maps2DController {
  private mouseState: MouseState;
  private dragStart: THREE.Vector2;
  private moveLeft: boolean;
  private moveRight: boolean;
  private moveDown: boolean;
  private moveUp: boolean;
  private viewHasChanged: boolean;

  constructor(
    private camera: THREE.OrthographicCamera,
    private domElement: Element
  ) {
    this.camera.zoom = 0.2;
    this.mouseState = MouseState.NONE;
    this.dragStart = new THREE.Vector2(0, 0);

    window.addEventListener(
      'keydown',
      (event) => this.onKeyDown(<KeyboardEvent>event),
      false
    );
    window.addEventListener(
      'keyup',
      (event) => this.onKeyUp(<KeyboardEvent>event),
      false
    );
    window.addEventListener(
      'mousewheel',
      (event) => this.onMouseWheel(<WheelEvent>event),
      false
    );
    this.domElement.addEventListener(
      'mousedown',
      (event) => this.onMouseDown(<MouseEvent>event),
      false
    );
  }

  public update(): boolean {
    let panByPixels = new THREE.Vector3(0, 0, 0);
    if (this.moveRight) {
      panByPixels.x += 4;
      this.viewHasChanged = true;
    }
    if (this.moveLeft) {
      panByPixels.x -= 4;
      this.viewHasChanged = true;
    }
    if (this.moveUp) {
      panByPixels.y += 4;
      this.viewHasChanged = true;
    }
    if (this.moveDown) {
      panByPixels.y -= 4;
      this.viewHasChanged = true;
    }

    this.camera.position.x += panByPixels.x / this.camera.zoom;
    this.camera.position.y += panByPixels.y / this.camera.zoom;
    this.camera.updateMatrix();

    // The camera's zoom is the number of pixels representing one meter.
    this.camera.updateProjectionMatrix();
    this.camera.updateMatrixWorld(true);
    let viewHasChanged = this.viewHasChanged;
    this.viewHasChanged = false;
    return viewHasChanged;
  }

  private setMoving(keyCode: string, state: boolean) {
    switch (keyCode) {
      case 'ArrowUp':
      case 'KeyW':
        this.moveUp = state;
        break;

      case 'ArrowDown':
      case 'KeyS':
        this.moveDown = state;
        break;

      case 'ArrowLeft':
      case 'KeyA':
        this.moveLeft = state;
        break;

      case 'ArrowRight':
      case 'KeyD':
        this.moveRight = state;
        break;
    }
  }

  private onKeyDown(event: KeyboardEvent) {
    event.stopPropagation();
    this.setMoving(event.code, true);
  }

  private onKeyUp(event: KeyboardEvent) {
    event.stopPropagation();
    this.setMoving(event.code, false);
  }

  private onMouseDown(event: MouseEvent) {
    event.preventDefault();
    if (event.button === 0) {
      this.mouseState = MouseState.DRAG;
      this.dragStart.set(event.clientX, event.clientY);
      this.domElement.addEventListener(
        'mousemove',
        (event) => this.onMouseMove(<MouseEvent>event),
        false
      );
      this.domElement.addEventListener(
        'mouseup',
        (event) => this.onMouseUp(<MouseEvent>event),
        false
      );
    }
  }

  private onMouseUp(event: MouseEvent) {
    this.domElement.removeEventListener(
      'mousemove',
      (event) => this.onMouseMove(<MouseEvent>event),
      false
    );
    this.domElement.removeEventListener(
      'mouseup',
      (event) => this.onMouseUp(<MouseEvent>event),
      false
    );
    this.mouseState = MouseState.NONE;
  }

  private onMouseMove(event: MouseEvent) {
    event.preventDefault();

    if (this.mouseState === MouseState.DRAG) {
      let dragEnd = new THREE.Vector2(event.clientX, event.clientY);
      let dragDelta = new THREE.Vector2(0, 0);
      dragDelta.subVectors(dragEnd, this.dragStart);
      this.camera.position.x -= dragDelta.x / this.camera.zoom;
      this.camera.position.y += dragDelta.y / this.camera.zoom;
      this.dragStart.copy(dragEnd);
      this.viewHasChanged = true;
    }
  }

  private onMouseWheel(event: WheelEvent) {
    event.preventDefault();
    let sign = event.deltaY < 0 ? 1 : -1;
    let scaleMultiplier = 1 + sign * 0.05;

    // Zoom around event.offsetX.
    let dx =
      (event.offsetX - this.domElement.clientWidth / 2) / this.camera.zoom;
    let dy =
      (event.offsetY - this.domElement.clientHeight / 2) / this.camera.zoom;
    this.camera.position.x += (scaleMultiplier - 1) * dx;
    this.camera.position.y -= (scaleMultiplier - 1) * dy;
    this.camera.zoom *= scaleMultiplier;
    this.viewHasChanged = true;
  }
}
