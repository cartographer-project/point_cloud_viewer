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

enum MouseState {
  NONE,
  ROTATE,
}

// A first person flight like controller.
export class FirstPersonController {
  private moveSpeed: number;
  private mouseState: MouseState;
  private rotateStart: THREE.Vector2;
  private moveBackward: boolean;
  private moveForward: boolean;
  private moveLeft: boolean;
  private moveRight: boolean;
  private moveDown: boolean;
  private moveUp: boolean;
  private thetaDelta: number;
  private phiDelta: number;

  constructor(private object: THREE.Object3D, private domElement: Element) {
    this.moveSpeed = 0.2;
    this.mouseState = MouseState.NONE;
    this.rotateStart = new THREE.Vector2(0, 0);
    this.thetaDelta = 0;
    this.phiDelta = 0;

    window.addEventListener(
        'keydown', event => this.onKeyDown(<KeyboardEvent>event), false);
    window.addEventListener(
        'keyup', event => this.onKeyUp(<KeyboardEvent>event), false);
    window.addEventListener(
        'mousewheel', event => this.onMouseWheel(<WheelEvent>event), false);
    this.domElement.addEventListener(
        'mousedown', event => this.onMouseDown(<MouseEvent>event), false);
  }

  public update(): boolean {
    let changed = false;
    let pan = new THREE.Vector3(0, 0, 0);
    if (this.moveRight) {
      pan.x += 1;
      changed = true;
    }
    if (this.moveLeft) {
      pan.x -= 1;
      changed = true;
    }
    if (this.moveBackward) {
      pan.z += 1;
      changed = true;
    }
    if (this.moveForward) {
      pan.z -= 1;
      changed = true;
    }
    if (this.moveUp) {
      pan.y += 1;
      changed = true;
    }
    if (this.moveDown) {
      pan.y -= 1;
      changed = true;
    }

    if (
      pan.lengthSq() > 0 ||
      Math.abs(this.thetaDelta) > 0 ||
      Math.abs(this.phiDelta)
    ) {
      changed = true;
      this.object.rotation.order = 'ZYX';

      this.object.matrixWorld.applyToVector3Array([pan.x, pan.y, pan.z]);
      this.object.translateOnAxis(pan.normalize(), this.moveSpeed);

      this.object.updateMatrix();

      let rot = new THREE.Matrix4().makeRotationZ(this.thetaDelta);
      let res = new THREE.Matrix4().multiplyMatrices(rot, this.object.matrix);
      this.object.quaternion.setFromRotationMatrix(res);

      this.object.rotation.x += this.phiDelta;
      this.object.updateMatrixWorld(true);

      this.thetaDelta = 0;
      this.phiDelta = 0;
    }
    return changed;
  }

  private setMoving(code: string, state: boolean) {
    switch (code) {
      case 'ArrowUp':
      case 'KeyW':
        this.moveForward = state;
        break;

      case 'ArrowDown':
      case 'KeyS':
        this.moveBackward = state;
        break;

      case 'ArrowLeft':
      case 'KeyA':
        this.moveLeft = state;
        break;

      case 'ArrowRight':
      case 'KeyD':
        this.moveRight = state;
        break;

      case 'KeyZ':
        this.moveDown = state;
        break;

      case 'KeyQ':
        this.moveUp = state;
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
      this.mouseState = MouseState.ROTATE;
      this.rotateStart.set(event.clientX, event.clientY);
      this.domElement.addEventListener(
          'mousemove', event => this.onMouseMove(<MouseEvent>event), false);
      this.domElement.addEventListener(
          'mouseup', event => this.onMouseUp(<MouseEvent>event), false);
    }
  }

  private onMouseUp(event: MouseEvent) {
    this.domElement.removeEventListener(
        'mousemove', event => this.onMouseMove(<MouseEvent>event), false);
    this.domElement.removeEventListener(
        'mouseup', event => this.onMouseUp(<MouseEvent>event), false);
    this.mouseState = MouseState.NONE;
  }

  private onMouseMove(event: MouseEvent) {
    event.preventDefault();

    if (this.mouseState === MouseState.ROTATE) {
      let rotateEnd = new THREE.Vector2(event.clientX, event.clientY);
      let rotateDelta = new THREE.Vector2(0, 0);
      rotateDelta.subVectors(rotateEnd, this.rotateStart);
      this.thetaDelta -=
          2 * Math.PI * rotateDelta.x / this.domElement.clientWidth;
      this.phiDelta -=
          2 * Math.PI * rotateDelta.y / this.domElement.clientHeight;
      this.rotateStart.copy(rotateEnd);
    }
  }

  private onMouseWheel(event: WheelEvent) {
    event.preventDefault();
    let sign = (event.wheelDelta < 0) ? -1 : 1;
    this.moveSpeed += sign * this.moveSpeed * 0.1;
    this.moveSpeed = Math.max(0.1, this.moveSpeed);
  }
}
