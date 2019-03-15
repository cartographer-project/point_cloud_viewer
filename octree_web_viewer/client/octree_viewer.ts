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

const KEY_L = 'L'.charCodeAt(0);

let VERTEX_SHADER = `
uniform float size;
uniform float gamma;
uniform float alpha;
uniform float edgeLength;
uniform vec3 min;

attribute vec3 color;

varying vec4 v_color;

void main() {
  vec3 corrected_color = pow(color / 255., vec3(1.0 / gamma));
  v_color = vec4(corrected_color, alpha);
  gl_Position = projectionMatrix * modelViewMatrix *
    vec4(position * edgeLength + min, 1.0);
  gl_PointSize = size;
}
`;

let FRAGMENT_SHADER = `
varying vec4 v_color;

void main() {
  gl_FragColor = v_color;
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
        public position: Float32Array | Uint16Array | Uint8Array,
        public normalizePosition: boolean,
        public color: Uint8Array
    ) { }
}

class NodeLoader {
    public load(
        scene: THREE.Scene,
        material: THREE.ShaderMaterial,
        nodes: NodeData[],
        octree_id: string
    ): Promise<void> {
        let query: string[] = [];

        for (const node of nodes) {
            query.push(`"${node.nodeName}"`);
        }
        const headers = new Headers();
        headers.append('Content-Type', 'application/json; charset=UTF-8');
        const request = new Request(`/nodes_data/${octree_id}/`, {
            method: 'POST',
            body: '[' + query.join(',') + ']',
            headers: headers,
            credentials: 'same-origin',
        });

        return window
            .fetch(request)
            .then((data) => data.arrayBuffer())
            .then((data) => {
                let view = new DataView(data);
                let currentEntry = 0;
                let numBytesRead = 0;
                while (nodes[currentEntry] !== undefined) {
                    let min_x = view.getFloat32(numBytesRead, true /* littleEndian */);
                    numBytesRead += 4;
                    let min_y = view.getFloat32(numBytesRead, true /* littleEndian */);
                    numBytesRead += 4;
                    let min_z = view.getFloat32(numBytesRead, true /* littleEndian */);
                    numBytesRead += 4;
                    let edgeLength = view.getFloat32(
                        numBytesRead,
                        true /* littleEndian */
                    );
                    numBytesRead += 4;

                    const numPoints = view.getUint32(
                        numBytesRead,
                        true /* littleEndian */
                    );
                    numBytesRead += 4;

                    const bytesPerCoordinate = view.getUint8(numBytesRead);
                    numBytesRead += 1;
                    if (numBytesRead % 4 != 0) {
                        numBytesRead += 4 - numBytesRead % 4;
                    }

                    let position: Float32Array | Uint16Array | Uint8Array;
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
                            console.log('Invalid bytesPerCoordinate: ', bytesPerCoordinate);
                    }
                    numBytesRead += numPoints * bytesPerCoordinate * 3;
                    if (numBytesRead % 4 != 0) {
                        numBytesRead += 4 - numBytesRead % 4;
                    }

                    let color = new Uint8Array(data, numBytesRead, numPoints * 3);
                    numBytesRead += numPoints * 3;
                    if (numBytesRead % 4 != 0) {
                        numBytesRead += 4 - numBytesRead % 4;
                    }

                    let render_data = new NodeRenderData(
                        new THREE.Vector3(min_x, min_y, min_z),
                        edgeLength,
                        position,
                        normalizePosition,
                        color
                    );
                    let node = nodes[currentEntry];
                    node.onDataLoaded(scene, material, render_data);
                    currentEntry += 1;
                }
            });
    }
}

class NodeData {
    public threePoints: THREE.Points;

    constructor(public nodeName: string) {
        this.threePoints = undefined;
    }

    public isUpToDate(): boolean {
        return this.threePoints !== undefined;
    }

    public onDataLoaded(
        scene: THREE.Scene,
        commonMaterial: THREE.ShaderMaterial,
        nodeRenderData: NodeRenderData
    ) {
        // If this node contains no points.
        if (nodeRenderData.position.length === 0 || this.isUpToDate()) {
            return;
        }

        const geometry = new THREE.BufferGeometry();
        // itemSize = 3 because there are 3 values (components) per vertex.
        geometry.addAttribute(
            'position',
            new THREE.BufferAttribute(
                nodeRenderData.position,
                3,
                nodeRenderData.normalizePosition
            )
        );
        geometry.addAttribute(
            'color',
            new THREE.BufferAttribute(nodeRenderData.color, 3)
        );

        // THREE can no longer figure out the bounding box or the bounding sphere of
        // this node, since the 'position' attribute does not contain it. So we
        // help it out.
        const SQR3 = 1.7320508075688772;
        geometry.boundingBox = new THREE.Box3(
            nodeRenderData.min,
            new THREE.Vector3(
                nodeRenderData.min.x + nodeRenderData.edgeLength,
                nodeRenderData.min.y + nodeRenderData.edgeLength,
                nodeRenderData.min.z + nodeRenderData.edgeLength
            )
        );
        geometry.boundingSphere = new THREE.Sphere(
            new THREE.Vector3(
                nodeRenderData.min.x + nodeRenderData.edgeLength / 2,
                nodeRenderData.min.y + nodeRenderData.edgeLength / 2,
                nodeRenderData.min.z + nodeRenderData.edgeLength / 2
            ),
            nodeRenderData.edgeLength / 2 * SQR3
        );

        let material = commonMaterial.clone();
        material.uniforms = {
            min: { value: nodeRenderData.min },
            edgeLength: { value: nodeRenderData.edgeLength },
            size: commonMaterial.uniforms['size'],
            alpha: commonMaterial.uniforms['alpha'],
            gamma: commonMaterial.uniforms['gamma'],
        };
        this.threePoints = new THREE.Points(geometry, material);
        scene.add(this.threePoints);
    }
}

export class OctreeViewer {
    // TODO(hrapp): These are only public, so we can wire up DAT to affect
    // material.size. If DAT supports callbacks, we can encapsulate this nicer.
    public material: THREE.ShaderMaterial;
    public maxLevelToDisplay: number;
    private octree_id: string;

    private loadedData: { [key: string]: NodeData } = {};
    private nodeLoader: NodeLoader;
    private batches: NodeData[][] = [];
    private currentlyLoading: number;
    private useTransparency: boolean;


    constructor(private scene: THREE.Scene, private onNewNodeData: () => void) {
        this.material = new THREE.ShaderMaterial({
            uniforms: {
                size: { value: 2 },
                alpha: { value: 1 },
                gamma: { value: 1 },
            },
            vertexShader: VERTEX_SHADER,
            fragmentShader: FRAGMENT_SHADER,
        });
        this.useTransparency = false;
        this.maxLevelToDisplay = 3;

        this.nodeLoader = new NodeLoader();
        this.currentlyLoading = 0;
        this.octree_id = 'init_id';
    }

    public load_new_tree(octree_id: string) {
        this.octree_id = octree_id;

    }

    public alphaChanged() {
        let newUseTransparency = this.material.uniforms['alpha'].value < 1;
        if (newUseTransparency != this.useTransparency) {
            this.material.transparent = newUseTransparency;
            this.scene.traverse(function (node) {
                if (node instanceof THREE.Points && node.material instanceof THREE.ShaderMaterial) {
                    node.material.transparent = newUseTransparency;
                }
            });
        }
        this.useTransparency = newUseTransparency;
    }

    public frustumChanged(matrix: THREE.Matrix4, width: number, height: number) {
        // ThreeJS is column major.
        const request = new Request(
            `/visible_nodes/${this.octree_id}/?width=${width}&height=${height}&matrix=${matrixToString(
                matrix
            )}`,
            {
                method: 'GET',
                credentials: 'same-origin',
            }
        );

        window
            .fetch(request)
            .then((data) => data.json())
            .then((nodes: any) => {
                this.nodesUpdate(nodes);
            });
    }

    public setMoving(moving: boolean) {
        for (const nodeId of Object.keys(this.loadedData)) {
            const threePoints = this.loadedData[nodeId].threePoints;
            if (threePoints !== undefined) {
                // If we are moving, only show points above a certain depth. Otherwise, show them all.
                if (moving) {
                    threePoints.visible = nodeId.length <= this.maxLevelToDisplay;
                } else {
                    threePoints.visible = true;
                }
            }
        }
    }

    private nodesUpdate(nodeIds: string[]) {
        const start = performance.now();
        this.batches = [];
        let currentBatch: NodeData[] = [];
        for (let nodeId of nodeIds) {
            let node = this.getOrCreate(nodeId);
            if (node.isUpToDate()) {
                continue;
            }

            currentBatch.push(node);
            if (currentBatch.length > 50) {
                this.batches.push(currentBatch);
                currentBatch = [];
            }
        }
        if (currentBatch.length > 0) {
            this.batches.push(currentBatch);
        }
        this.handleNextBatch();
        console.log(`nodeUpdate took ${performance.now() - start}ms.`);
    }

    private handleNextBatch() {
        if (this.batches.length == 0 || this.currentlyLoading > 2) {
            return;
        }
        this.currentlyLoading += 1;
        this.nodeLoader
            .load(this.scene, this.material, this.batches.shift(), this.octree_id)
            .then(() => {
                this.currentlyLoading -= 1;
                this.onNewNodeData();
                this.handleNextBatch();
            });
    }

    private getOrCreate(nodeName: string): NodeData {
        if (this.loadedData[nodeName] === undefined) {
            this.loadedData[nodeName] = new NodeData(nodeName);
        }
        return this.loadedData[nodeName];
    }
}
