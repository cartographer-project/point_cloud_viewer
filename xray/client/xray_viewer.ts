'use strict';

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
  public plane: THREE.Mesh;
  public inScene: boolean;

  constructor(public id: string, public boundingRect: any) {
    this.inScene = false;
  }

  public setTexture(texture: THREE.Texture) {
    var geometry = new THREE.PlaneGeometry(this.boundingRect['edge_length'], this.boundingRect['edge_length'], 1);
    geometry.applyMatrix(new THREE.Matrix4().makeTranslation(this.boundingRect['edge_length'] / 2., - this.boundingRect['edge_length'] / 2., 0.));
    var material = new THREE.MeshBasicMaterial({ map: texture, side: THREE.DoubleSide });
    this.plane = new THREE.Mesh(geometry, material);
    this.plane.scale.y = -1;
    this.plane.position.set(this.boundingRect['min_x'], this.boundingRect['min_y'], 0.);
  }
};

export class XRayViewer {
  private nodes: { [key: string]: NodeData } = {};
  private currentlyLoading: number;
  private nodesToLoad: string[] = [];

  constructor(private scene: THREE.Scene, private meta: any) {
    this.currentlyLoading = 0;
  }

  public frustumChanged(matrix: THREE.Matrix4, pixelsPerMeter: number) {
      // Figure out which view level represents our current zoom.
      let full_size_considering_zoom = pixelsPerMeter * this.meta['bounding_rect']['edge_length'];
      let level = 0;
      let edge_length_px_for_level = this.meta['tile_size'];
      while (edge_length_px_for_level < full_size_considering_zoom && level < this.meta['deepest_level']) {
        edge_length_px_for_level *= 2;
        level += 1;
      }

    // ThreeJS is column major.
    const request = new Request(
      `/nodes_for_level?level=${level}&matrix=${matrixToString(matrix)}`,
      {
        method: 'GET',
        credentials: 'same-origin',
      });

    window.fetch(request).then(data => data.json()).then((nodes: any) => {
      this.nodesUpdate(nodes);
    });

  }

  private nodesUpdate(nodes: any) {
    this.nodesToLoad = [];
    for (var i = 0, len = nodes.length; i < len; i++) {
      let node = this.getOrCreate(nodes[i]['id'], nodes[i]['bounding_rect']);
      if (node.plane !== undefined) {
        this.swapIn(node.id);
        continue;
      }
      this.nodesToLoad.push(node.id);
    }
    this.loadNext();
  }

  private loadNext() {
    if (this.nodesToLoad.length == 0 || this.currentlyLoading > 2) {
      return;
    }
    this.currentlyLoading += 1;
    let nodeId = this.nodesToLoad.shift();
    new THREE.TextureLoader().load("/node_image/" + nodeId, (texture) => {
      this.currentlyLoading -= 1;
      this.loadNext();
      this.nodes[nodeId].setTexture(texture);
      this.swapIn(nodeId);
    });
  }

  private getOrCreate(nodeId: string, boundingRect: any): NodeData {
    if (this.nodes[nodeId] === undefined) {
      this.nodes[nodeId] = new NodeData(nodeId, boundingRect);
    }
    return this.nodes[nodeId];
  }

  private swapIn(nodeId: string) {
    if (this.nodes[nodeId].inScene) {
      return;
    }
    console.log("Swapping in: ", nodeId);
    // Swap out parents and children. 
    // TODO(sirver): We never drop any nodes, so memory is used unbounded.
    // TODO(sirver): Parent should only be swapped out if all children are loaded.
    if (nodeId !== "r") {
      let parentId = nodeId.substring(0, nodeId.length - 1);
      if (this.nodes[parentId] !== undefined && this.nodes[parentId].inScene) {
        console.log("Swapping out: ", nodeId);
        this.scene.remove(this.nodes[parentId].plane);
        this.nodes[parentId].inScene = false;
      }
    }
    for (let i = 0; i < 4; i++) {
      let childId = nodeId + i;
      if (this.nodes[childId] !== undefined && this.nodes[childId].inScene) {
        console.log("Swapping out: ", childId);
        this.scene.remove(this.nodes[childId].plane);
        this.nodes[childId].inScene = false;
      }
    }
    this.scene.add(this.nodes[nodeId].plane);
    this.nodes[nodeId].inScene = true;
  }

};

