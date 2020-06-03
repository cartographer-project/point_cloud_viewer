'use strict';

import * as THREE from 'three';

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

class NodeData {
  public plane: THREE.Mesh = null;
  public inScene: boolean;

  constructor(public id: string, public boundingRect: any) {
    this.inScene = false;
  }

  public setTexture(texture: THREE.Texture) {
    if (this.plane !== null) {
      // This can happen when we were created, a load was triggered and
      // then the frustum changed. We will then possibly be scheduled for
      // load again. An alternative way of avoiding that is to carry a
      // 'currentlyRequested' flag on this object and not request this tile
      // if it is set.
      return;
    }
    let geometry = new THREE.PlaneGeometry(
      this.boundingRect['edge_length'],
      this.boundingRect['edge_length'],
      1
    );
    geometry.applyMatrix4(
      new THREE.Matrix4().makeTranslation(
        this.boundingRect['edge_length'] / 2,
        this.boundingRect['edge_length'] / 2,
        0
      )
    );
    let material = new THREE.MeshBasicMaterial({
      map: texture,
      side: THREE.DoubleSide,
      transparent: true
    });
    this.plane = new THREE.Mesh(geometry, material);
    this.plane.position.set(
      this.boundingRect['min_x'],
      this.boundingRect['min_y'],
      -100
    );
  }
}

export class XRayViewer {
  private nodes: { [key: string]: NodeData } = {};
  private currentlyLoading: number;
  private nodesToLoad: string[] = [];
  private nextNodesForLevelQueryId: number;
  private displayLevel: number;
  private meta: any; // Will be undefined until we have loaded the metadata.
  private isDisposed = false;

  constructor(private scene: THREE.Scene, private prefix: string) {
    this.currentlyLoading = 0;
    this.nextNodesForLevelQueryId = 0;
    this.displayLevel = 0;
    window
      .fetch(`${this.prefix}/meta`)
      .then((data) => data.json())
      .then((meta: any) => {
        this.meta = meta;
      });
  }

  public dispose() {
    this.isDisposed = true;
    for (const [nodeId, node] of Object.entries(this.nodes)) {
      if (this.nodes[nodeId].inScene) {
        this.removeFromScene(this.nodes[nodeId]);
      }
    }
    this.nodesToLoad = [];
  }

  private removeFromScene(node: NodeData) {
    // TODO(sirver): This does not dispose of texture and plane geometry.
    this.scene.remove(node.plane);
    node.inScene = false;
  }

  public isInitialized(): boolean {
    return this.meta !== undefined && !this.isDisposed;
  }

  public frustumChanged(matrix: THREE.Matrix4, pixelsPerMeter: number) {
    if (!this.isInitialized()) {
      return;
    }
    // Figure out which view level represents our current zoom.
    let full_size_considering_zoom =
      pixelsPerMeter * this.meta['bounding_rect']['edge_length'];
    let level = 0;
    let edge_length_px_for_level = this.meta['tile_size'];
    while (
      edge_length_px_for_level < full_size_considering_zoom &&
      level < this.meta['deepest_level']
    ) {
      edge_length_px_for_level *= 2;
      level += 1;
    }
    this.displayLevel = level;

    this.nextNodesForLevelQueryId++;
    let queryId = this.nextNodesForLevelQueryId;
    // We encode the matrix column major.
    const matrixStr = matrixToString(matrix);
    window
      .fetch(
        `${this.prefix}/nodes_for_level?level=${level}&matrix=${matrixStr}`
      )
      .then((data) => data.json())
      .then((nodes: any) => {
        // Ignore responses that arrive after we've already issued a new request.
        // TODO(sirver): Look into axios for canceling running requests.
        if (this.nextNodesForLevelQueryId === queryId) {
          this.nodesUpdate(nodes);
        }
      });
  }

  private nodesUpdate(nodes: any) {
    this.nodesToLoad = [];
    for (let i = 0, len = nodes.length; i < len; i++) {
      let node = this.getOrCreate(nodes[i]['id'], nodes[i]['bounding_rect']);
      if (node.plane !== null) {
        this.swapIn(node.id);
        continue;
      }
      this.nodesToLoad.push(node.id);
    }
    this.loadNext();
    if (this.currentlyLoading === 0) {
      // Done loading
      this.onlyShowDisplayLevel();
    }
  }

  private loadNext() {
    if (this.nodesToLoad.length == 0 || this.currentlyLoading > 2) {
      return;
    }
    this.currentlyLoading += 1;
    let nodeId = this.nodesToLoad.shift();
    new THREE.TextureLoader().load(
      `${this.prefix}/node_image/${nodeId}`,
      (texture) => {
        this.currentlyLoading -= 1;
        this.loadNext();
        const level = nodeId.length - 1;
        // Let's show all the beautiful pixels on the lowest zoom level.
        if (level == this.meta['deepest_level']) {
          texture.magFilter = THREE.NearestFilter;
        }
        this.nodes[nodeId].setTexture(texture);
        this.swapIn(nodeId);
        if (this.currentlyLoading === 0) {
          // Done loading
          this.onlyShowDisplayLevel();
        }
      }
    );
  }

  private getOrCreate(nodeId: string, boundingRect: any): NodeData {
    if (this.nodes[nodeId] === undefined) {
      this.nodes[nodeId] = new NodeData(nodeId, boundingRect);
    }
    return this.nodes[nodeId];
  }

  private onlyShowDisplayLevel() {
    for (const [nodeId, node] of Object.entries(this.nodes)) {
      let level = nodeId.length - 1;
      if (node.inScene && level !== this.displayLevel) {
        this.removeFromScene(node);
      }
    }
  }

  private swapIn(nodeId: string) {
    if (this.isDisposed) {
      return;
    }
    let level = nodeId.length - 1;
    if (this.nodes[nodeId].inScene || level !== this.displayLevel) {
      return;
    }
    // TODO(sirver): We never drop any nodes, so memory is used unbounded.
    this.scene.add(this.nodes[nodeId].plane);
    this.nodes[nodeId].inScene = true;
  }
}
