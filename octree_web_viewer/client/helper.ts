import * as THREE from 'three';

function assert(cond: boolean, message: string) {
    if (!cond) {
        throw new Error(`Assertion failed: ${message}`);
    }
}

// Object types that don't need to be disposed.
const noDisposeRequiredTypes = ['Object3D', 'Scene', 'PerspectiveCamera', 'DirectionalLight'];

// Basic object types that can be disposed of in a uniform way.
type BasicDisposable = THREE.Mesh | THREE.Line | THREE.LineSegments | THREE.Points;
const basicDisposableTypes = ['Mesh', 'Line', 'LineSegments', 'Points'];

function isBasicDisposable(obj: THREE.Object3D): obj is BasicDisposable {
    return basicDisposableTypes.some(t => obj.type === t);
}

function isGroup(obj: THREE.Object3D) {
    return obj.type === 'Group';
}

function noDisposeRequired(obj: THREE.Object3D) {
    return noDisposeRequiredTypes.some(t => obj.type === t);
}

function disposeMaterial(mat: THREE.Material) {
    if (mat instanceof THREE.MeshBasicMaterial || mat instanceof THREE.MeshLambertMaterial) {
        if (mat.map) mat.map.dispose();
        if (mat.aoMap) mat.aoMap.dispose();
        if (mat.specularMap) mat.specularMap.dispose();
        if (mat.alphaMap) mat.alphaMap.dispose();
        if (mat.envMap) mat.envMap.dispose();
    } else if (mat instanceof THREE.PointsMaterial) {
        if (mat.map) mat.map.dispose();
    } else if (mat instanceof THREE.LineBasicMaterial) {
        // Nothing to do.
    } else if (mat instanceof THREE.ShaderMaterial) {
        // Nothing to do
    } else {
        assert(false, `Unhandled material type: ${mat.constructor.name}`);
    }
}

function doBasicDispose(obj: BasicDisposable) {
    const { geometry, material } = obj;
    if (geometry) {
        geometry.dispose();
    }

    if (Array.isArray(material)) {
        material.forEach(disposeMaterial);
    } else if (material) {
        disposeMaterial(material);
    }
}

function disposeGroup(obj: THREE.Group) {
    obj.children.forEach(disposeNode);
}

function disposeNode(node: THREE.Object3D) {
    if (isBasicDisposable(node)) {
        doBasicDispose(node);
    } else if (isGroup(node)) {
        disposeGroup(node as THREE.Group);
    } else if (noDisposeRequired(node)) {
        // Do nothing.
    } else {
        assert(false, `Unhandled node type '${node.type}'`);
    }
}

// Recursively dispose all of the disposable objects reachable from `root`.
// Rather than trying to exhaustively handle every possible type of object, this function is
// restricted to the types of objects we use. To handle a new type of object, look at its type
// definition (https://github.com/DefinitelyTyped/DefinitelyTyped/blob/master/types/three/three-core.d.ts),
// and watch out for members that have a `dispose` method.
export default function disposeSceneGraph(root: THREE.Object3D) {
    root.traverse(disposeNode);
}
