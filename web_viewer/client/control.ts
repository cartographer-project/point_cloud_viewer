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

const KEY_LEFT = 37;
const KEY_UP = 38;
const KEY_RIGHT = 39;
const KEY_DOWN = 40;

const KEY_A = 'A'.charCodeAt(0);
const KEY_S = 'S'.charCodeAt(0);
const KEY_D = 'D'.charCodeAt(0);
const KEY_W = 'W'.charCodeAt(0);
const KEY_Z = 'Z'.charCodeAt(0);
const KEY_Q = 'Q'.charCodeAt(0);

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

  public update() {
    let pan = new THREE.Vector3(0, 0, 0);
    if (this.moveRight) {
      pan.x += 1;
    }
    if (this.moveLeft) {
      pan.x -= 1;
    }
    if (this.moveBackward) {
      pan.z += 1;
    }
    if (this.moveForward) {
      pan.z -= 1;
    }
    if (this.moveUp) {
      pan.y += 1;
    }
    if (this.moveDown) {
      pan.y -= 1;
    }

    if (
      pan.lengthSq() > 0 ||
      Math.abs(this.thetaDelta) > 0 ||
      Math.abs(this.phiDelta)
    ) {
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
  }

  private setMoving(keyCode: number, state: boolean) {
    switch (keyCode) {
      case KEY_UP:
      case KEY_W:
        this.moveForward = state;
        break;

      case KEY_DOWN:
      case KEY_S:
        this.moveBackward = state;
        break;

      case KEY_LEFT:
      case KEY_A:
        this.moveLeft = state;
        break;

      case KEY_RIGHT:
      case KEY_D:
        this.moveRight = state;
        break;

      case KEY_Z:
        this.moveDown = state;
        break;

      case KEY_Q:
        this.moveUp = state;
        break;
    }
  }

  private onKeyDown(event: KeyboardEvent) {
    event.stopPropagation();
    this.setMoving(event.keyCode, true);
  }

  private onKeyUp(event: KeyboardEvent) {
    event.stopPropagation();
    this.setMoving(event.keyCode, false);
  }

  private onMouseDown(event: MouseEvent) {
    event.preventDefault();
    if (event.button === 0) {
      this.mouseState = MouseState.ROTATE;
      this.rotateStart.set(event.clientX, event.clientY);
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
    let sign = event.wheelDelta < 0 ? -1 : 1;
    this.moveSpeed += sign * this.moveSpeed * 0.1;
    this.moveSpeed = Math.max(0.1, this.moveSpeed);
  }
}
