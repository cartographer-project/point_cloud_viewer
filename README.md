# Point viewer

[![build status](https://travis-ci.org/googlecartographer/point_cloud_viewer.svg?branch=master)](https://travis-ci.org/googlecartographer/point_cloud_viewer) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://github.com/googlecartographer/point_cloud_viewer/blob/master/LICENSE)

This is a standalone project to make viewing massive point clouds easy and convenient.
It was build to serve the needs of the [Cartographer](https://github.com/googlecartographer) project, but is useful in its own right.

## Building

The project consist of a root crate that can build and read octrees on disk and viewer binaries to visualize the data. For Mac OS X, we assume below that you've installed [Homebrew](https://brew.sh).

- Install Rust: `curl https://sh.rustup.rs -sSf | sh`. See <https://rustup.rs> for details.
- Install protobuf > 3.0. See [ci/install_proto3.sh](https://github.com/googlecartographer/point_cloud_viewer/blob/master/ci/install_proto3.sh) for Linux or run `brew install protobuf` on Mac OS X.
- Install the rust gRPC protobuf plugin: `cargo install grpcio-compiler` and
  make sure it is in your `$PATH`. This has more dependencies, it requires cmake
  and go. On mac: `brew install cmake go`.

### Creating Octrees

In the root of the repo, run `cargo build --release`.
Then use `target/release/build_octree` to generate an octree out of a PLY file (`binary_little_endian` format).

### SDL client

This is a native client using [SDL2](https://libsdl.org).

1. Install SDL2. For example, on Mac `brew install sdl2`. 
2. Change to the sdl viewer's directory: `cd sdl_viewer/`. 
3. Build with `cargo build --release`. 
4. Run with `../target/release/sdl_viewer <octree directory>`.

In the point cloud viewer, navigate with the keyboard or with the mouse or touchpad. Dragging while pressing the left mouse button rotates, dragging while pressing the right mouse button pans the view. The following keys are bound:

| Key                | Action                        |
| ------------------ | ----------------------------- |
| W                  | Move forward                  |
| A                  | Move left                     |
| S                  | Move backwards                |
| D                  | Move right                    |
| Q                  | Move up                       |
| Z                  | Move down                     |
| Up                 | Turn up                       |
| Left               | Turn left                     |
| Down               | Turn down                     |
| Right              | Move right                    |
| 0                  | Increase points size          |
| 9                  | Decrease points size          |
| 8                  | Brighten scene                |
| 7                  | Darken scene                  |
| O                  | Show octree nodes             |
| Shift + Ctrl + 0-9 | Save current camera position. |
| Ctrl + 0-9         | Load saved camera position.   |

Saved camera positions are persisted in the octree directory and will therefore live through restarts of the program.

### Web Viewer

The `octree_web_viewer` consists of [TypeScript](https://www.typescriptlang.org) code running in the browser and a web server binary.

To build,

1. Change into the web viewer's client directory: `cd octree_web_viewer/client`.
2. Install npm. We strongly suggest using [nvm](https://github.com/creationix/nvm). On Mac `brew install nvm`. 
3. Install node version 8: `nvm install 8`. Change to the web viewer's client directory: `cd client`, then set node version to 8: `nvm use 8`. 
4. Install javascript dependencies: `npm install`.
5. Build the client: `npm run build`.

Then build the server: `cargo build --release`.
Serve up the octree using `../target/release/octree_web_viewer <octree directory>`, open Chrome to <http://localhost:5433>, navigate with WASD and left-click-drag on the mouse.
The mouse wheel adjusts movement speed.

The client files (HTML and JavaScript) are embedded in the `octree_web_viewer` binary, so it is fully stand alone.

## Prior art

This work was inspired through the following projects.

- [Potree](http://potree.org)
- [Megatree](http://wiki.ros.org/megatree)
