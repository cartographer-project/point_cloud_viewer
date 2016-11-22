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

import {now} from './main';

const KEY_L = 'L'.charCodeAt(0);

let VERTEX_SHADER = `
uniform float size;
attribute vec3 color;

varying vec3 v_color;  // 'varying' vars are passed to the fragment shader

void main() {
  v_color = color;   // pass the color to the fragment shader
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  gl_PointSize = size;
}
`;

let FRAGMENT_SHADER = `
varying vec3 v_color;

void main() {
  gl_FragColor = vec4(v_color, 1.);
}
`;

function matrixToString(m: THREE.Matrix4): string {
  const me = m.elements;
  return [
    me[0].toFixed(10),
    me[1].toFixed(10),
    me[2].toFixed(10),
    me[3].toFixed(10),
    me[4].toFixed(10),
    me[5].toFixed(10),
    me[6].toFixed(10),
    me[7].toFixed(10),
    me[8].toFixed(10),
    me[9].toFixed(10),
    me[10].toFixed(10),
    me[11].toFixed(10),
    me[12].toFixed(10),
    me[13].toFixed(10),
    me[14].toFixed(10),
    me[15].toFixed(10),
  ].join(',');
}

class NodePoints {
  constructor(public points: Float32Array, public colors: Float32Array) {}
}

class NodeLoader {
  public load(scene: THREE.Scene, material: THREE.ShaderMaterial, entries: [
    NodeData, number
  ][]): Promise<void> {
    let query: string[] = [];

    for (const entry of entries) {
      entry[0].startFetching(entry[1]);
      query.push(`["${entry[0].nodeName}",${entry[1]}]`);
    }
    const headers = new Headers();
    headers.append('Content-Type', 'application/json; charset=UTF-8');
    const request = new Request('/nodes_data', {
      method: 'POST',
      body: '[' + query.join(',') + ']',
      headers: headers,
      credentials: 'same-origin',
    });

    return window.fetch(request).then(data => data.arrayBuffer()).then(data => {
      let view = new DataView(data);
      let numBytesRead = 0;
      let currentEntry = 0;
      while (numBytesRead < data.byteLength) {
        const numBytesInBlob =
            view.getUint32(numBytesRead, true /* littleEndian */);
        numBytesRead += 4;
        const numPoints = numBytesInBlob / 24;
        let points = new NodePoints(
            new Float32Array(data, numBytesRead, numPoints * 3),
            new Float32Array(
                data, numBytesRead + numPoints * 12, numPoints * 3));
        let entry = entries[currentEntry];
        entry[0].newData(scene, material, entry[1], points);
        numBytesRead += numPoints * 24;
        currentEntry += 1;
      }
    });
  }
}

class NodeData {
  private displayedLevel: number;
  private fetchingLevel: number;
  private threePoints: THREE.Points;

  constructor(public nodeName: string) {
    this.displayedLevel = -1;
    this.fetchingLevel = -1;
    this.threePoints = undefined;
  }

  public isUpToDate(lod: number) {
    if (this.fetchingLevel === lod) {
      return true;
    }
    if (this.fetchingLevel === -1 && this.displayedLevel === lod) {
      return true;
    }
    return false;
  }

  public startFetching(lod: number) { this.fetchingLevel = lod; }

  public newData(
      scene: THREE.Scene, material: THREE.ShaderMaterial, lod: number,
      nodePoints: NodePoints) {
    // If this node contains no points.
    if (nodePoints.points.length === 0) {
      return;
    }

    if (lod !== this.fetchingLevel) {
      return;
    }
    this.fetchingLevel = -1;

    if (this.displayedLevel === lod) {
      return;
    }
    this.displayedLevel = lod;

    if (this.threePoints !== undefined) {
      scene.remove(this.threePoints);
    }

    const geometry = new THREE.BufferGeometry();
    // itemSize = 3 because there are 3 values (components) per vertex
    geometry.addAttribute(
        'position', new THREE.BufferAttribute(nodePoints.points, 3));
    geometry.addAttribute(
        'color', new THREE.BufferAttribute(nodePoints.colors, 3));
    this.threePoints = new THREE.Points(geometry, material);
    scene.add(this.threePoints);
  }
};

export class OctreeViewer {
  // TODO(hrapp): These are only public, so we can wire up DAT to affect
  // material.size. If DAT supports callbacks, we can encapsulate this nicer.
  public material: THREE.ShaderMaterial;
  public useLod: boolean;

  private loadedData: {[key: string]: NodeData} = {};
  private nodeLoader: NodeLoader;
  private batches: [NodeData, number][][] = [];
  private currentlyLoading: number;

  constructor(private scene: THREE.Scene) {
    this.material = new THREE.ShaderMaterial({
      uniforms: {
        size: { value: 1e-2 },
      },
      vertexShader: VERTEX_SHADER,
      fragmentShader: FRAGMENT_SHADER,
    });

    window.addEventListener(
        'keydown', event => this.onKeyDown(<KeyboardEvent>event), false);
    this.useLod = true;
    this.nodeLoader = new NodeLoader();
    this.currentlyLoading = 0;
  }

  private onKeyDown(event: KeyboardEvent) {
    switch (event.keyCode) {
      case KEY_L:
        this.useLod = !this.useLod;
        break;
    }
  }

  public frustumChanged(matrix: THREE.Matrix4, width: number, height: number) {
    // ThreeJS is column major.
    const request = new Request(
        `/visible_nodes?use_lod=${this.useLod ? 1 : 0}&width=${width}&height=${height}&matrix=${matrixToString(matrix)}`,
        {
          method: 'GET',
          credentials: 'same-origin',
        });

    window.fetch(request).then(data => data.json()).then(nodes => {
      this.nodesUpdate(nodes);
    });
  }

  private nodesUpdate(nodes: [string, number][]) {
    const start = now();
    this.batches = [];
    let currentBatch: [NodeData, number][] = [];
    for (let entry of nodes) {
      let node = this.getOrCreate(entry[0]);
      if (node.isUpToDate(entry[1])) {
        continue;
      }

      currentBatch.push([node, entry[1]]);
      if (currentBatch.length > 50) {
        this.batches.push(currentBatch);
        currentBatch = [];
      }
    }
    if (currentBatch.length > 0) {
      this.batches.push(currentBatch)
    }
    this.handleNextBatch();
    console.log(`nodeUpdate took ${now() - start}ms.`);
  }

  private handleNextBatch() {
    if (this.batches.length == 0 || this.currentlyLoading > 2) {
      return;
    }
    this.currentlyLoading += 1;
    this.nodeLoader.load(this.scene, this.material, this.batches.shift())
        .then(() => {
          this.currentlyLoading -= 1;
          this.handleNextBatch();
        })
  }

  private getOrCreate(nodeName: string): NodeData {
    if (this.loadedData[nodeName] === undefined) {
      this.loadedData[nodeName] = new NodeData(nodeName);
    }
    return this.loadedData[nodeName];
  }
};
