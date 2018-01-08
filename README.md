# Point viewer

[![build status](https://travis-ci.org/googlecartographer/point_cloud_viewer.svg?branch=master)](https://travis-ci.org/googlecartographer/point_cloud_viewer) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://github.com/googlecartographer/point_cloud_viewer/blob/master/LICENSE)

This is a standalone project to make viewing massive point clouds easy and convenient.
It was build to serve the needs of the [Cartographer](https://github.com/googlecartographer) project, but is useful in its own right.

## Building

The project consist of a root crate that can build and read octrees on disk and viewer binaries to visualize the data.

First, install Rust: `curl https://sh.rustup.rs -sSf | sh`.
See <https://rustup.rs> for details.

### Creating Octrees

In the root of the repo, run `cargo build --release`.
Then use `target/release/build_octree` to generate an octree out of a PLY file.

### Web Viewer

The `web_viewer` consists of [TypeScript](https://www.typescriptlang.org) code running in the browser and a web server binary.

To build,

1. Change into the web viewer's client directory: `cd web_viewer/client`.
2. Install npm. We strongly suggest using [nvm](https://github.com/creationix/nvm). On Mac `brew install nvm`. 
3. Install node version 8: `nvm install 8`. Change to the web viewer's client directory: `cd client`, then set node version to 8: `nvm use 8`. 
4. Install javascript dependencies: `npm install`.
5. Build the client: `npm run build`.

Then build the server: `cargo build --release`.
Serve up the octree using `web_viewer/target/release/web_viewer <octree directory>`, open Chrome to <http://localhost:5433>, navigate with WASD and left-click-drag on the mouse.
The mouse wheel adjusts movement speed.

The client files (HTML and JavaScript) are embedded in the `web_viewer` binary, so it is fully stand alone.

### SDL client (experimental)

This is a native client using [SDL2](https://libsdl.org).

1. Install SDL2. For example, on Mac `brew install sdl2`. (This requires Homebrew, https://brew.sh)
2. Change to the sdl viewer's directory: `cd sdl_viewer/`. 
3. Build with `cargo build --release`. 
4. Run with `target/release/sdl_viewer <octree directory>`.

In the point cloud viewer, navigate with keys A, W, D, S. Rotate using touchpad. Keys 9 and 0 make points smaller and larger, keys 7 and 8 make points darker and brighter. 


## Prior art

This work was inspired through the following projects.

- [Potree](http://potree.org)
- [Megatree](http://wiki.ros.org/megatree)
