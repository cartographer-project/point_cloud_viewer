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
uniform float edgeLength;
uniform vec3 min;

attribute vec3 color;

varying vec3 v_color;

void main() {
  v_color = color / 255.;   // pass the color to the fragment shader
  gl_Position = projectionMatrix * modelViewMatrix *
    vec4(position * edgeLength + min, 1.0);
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

class NodeRenderData {
  constructor(
      public min: THREE.Vector3,
      public edgeLength: number,
      public position: Float32Array|Uint16Array|Uint8Array,
      public normalizePosition: boolean,
      public color: Uint8Array) {}
}

class NodeLoader {
  public load(scene: THREE.Scene,
              material: THREE.ShaderMaterial,
              entries: [NodeData, number][]): Promise<void> {
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
      let currentEntry = 0;
      let numBytesRead = 0;
      while (entries[currentEntry] !== undefined) {
        let min_x = view.getFloat32(numBytesRead, true /* littleEndian */);
        numBytesRead += 4;
        let min_y = view.getFloat32(numBytesRead, true /* littleEndian */);
        numBytesRead += 4;
        let min_z = view.getFloat32(numBytesRead, true /* littleEndian */);
        numBytesRead += 4;
        let edgeLength = view.getFloat32(numBytesRead, true /* littleEndian */);
        numBytesRead += 4;

        const numPoints =
            view.getUint32(numBytesRead, true /* littleEndian */);
        numBytesRead += 4;

        const bytesPerCoordinate = view.getUint8(numBytesRead);
        numBytesRead += 1;
        if (numBytesRead % 4 != 0) {
          numBytesRead += 4 - (numBytesRead % 4);
        }

        let position: Float32Array|Uint16Array|Uint8Array;
        let normalizePosition: boolean;
        switch (bytesPerCoordinate) {
          case 4:
            position = new Float32Array(data, numBytesRead, numPoints * 3);
            normalizePosition = false;
            break;

          case 2:
            position = new Uint16Array(data, numBytesRead, numPoints * 3);
            normalizePosition = true;
            break;

          case 1:
            position = new Uint8Array(data, numBytesRead, numPoints * 3);
            normalizePosition = true;
            break;

          default:
            console.log("Invalid bytesPerCoordinate: ", bytesPerCoordinate);
        }
        numBytesRead += numPoints * bytesPerCoordinate * 3;
        if (numBytesRead % 4 != 0) {
          numBytesRead += 4 - (numBytesRead % 4);
        }

        let color = new Uint8Array(data, numBytesRead, numPoints * 3);
        numBytesRead += numPoints * 3;
        if (numBytesRead % 4 != 0) {
          numBytesRead += 4 - (numBytesRead % 4);
        }

        let render_data = new NodeRenderData(
            new THREE.Vector3(min_x, min_y, min_z), edgeLength, position,
            normalizePosition, color);
        let entry = entries[currentEntry];
        entry[0].newData(scene, material, entry[1], render_data);
        currentEntry += 1;
      }
    });
  }
}

class NodeData {
  private diplayedLevelOfDetail: number;
  private fetchingLevelOfDetail: number;
  private threePoints: THREE.Points;

  constructor(public nodeName: string) {
    this.diplayedLevelOfDetail = -1;
    this.fetchingLevelOfDetail = -1;
    this.threePoints = undefined;
  }

  public isUpToDate(lod: number) {
    if (this.fetchingLevelOfDetail === lod) {
      return true;
    }
    if (this.fetchingLevelOfDetail === -1 && this.diplayedLevelOfDetail === lod) {
      return true;
    }
    return false;
  }

  public startFetching(lod: number) { this.fetchingLevelOfDetail = lod; }

  public newData(
      scene: THREE.Scene, commonMaterial: THREE.ShaderMaterial, lod: number,
      nodeRenderData: NodeRenderData) {
    // If this node contains no points.
    if (nodeRenderData.position.length === 0) {
      return;
    }

    if (lod !== this.fetchingLevelOfDetail) {
      return;
    }
    this.fetchingLevelOfDetail = -1;

    if (this.diplayedLevelOfDetail === lod) {
      return;
    }
    this.diplayedLevelOfDetail = lod;

    if (this.threePoints !== undefined) {
      scene.remove(this.threePoints);
    }

    const geometry = new THREE.BufferGeometry();
    // itemSize = 3 because there are 3 values (components) per vertex.
    geometry.addAttribute(
        'position', new THREE.BufferAttribute(nodeRenderData.position, 3, nodeRenderData.normalizePosition));
    geometry.addAttribute(
        'color', new THREE.BufferAttribute(nodeRenderData.color, 3));

    // THREE can no longer figure out the bounding box or the bounding sphere of
    // this node, since the 'position' attribute does not contain it. So we
    // help it out.
    geometry.boundingBox = new THREE.Box3(
        nodeRenderData.min,
        new THREE.Vector3(
            nodeRenderData.min.x + nodeRenderData.edgeLength,
            nodeRenderData.min.y + nodeRenderData.edgeLength,
            nodeRenderData.min.z + nodeRenderData.edgeLength));
    geometry.boundingSphere = new THREE.Sphere(
        new THREE.Vector3(
            nodeRenderData.min.x + nodeRenderData.edgeLength / 2,
            nodeRenderData.min.y + nodeRenderData.edgeLength / 2,
            nodeRenderData.min.z + nodeRenderData.edgeLength / 2),
        nodeRenderData.edgeLength / 2
    );

    let material = commonMaterial.clone();
    material.uniforms = {
      min: { value: nodeRenderData.min },
      edgeLength: { value: nodeRenderData.edgeLength },
      size: commonMaterial.uniforms.size,
    };
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
        size: { value: 2. },
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
