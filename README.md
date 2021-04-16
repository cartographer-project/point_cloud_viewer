# Point viewer

[![build status](https://travis-ci.org/cartographer-project/point_cloud_viewer.svg?branch=master)](https://travis-ci.org/cartographer-project/point_cloud_viewer) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://github.com/cartographer-project/point_cloud_viewer/blob/master/LICENSE)

This is a standalone project to make viewing massive point clouds easy and convenient.
It was build to serve the needs of the [Cartographer](https://github.com/cartographer-project) project, but is useful in its own right.

## Building

The project consist of a root crate that can build and read octrees on disk and viewer binaries to visualize the data. For Mac OS X, we assume below that you've installed [Homebrew](https://brew.sh).

- Install Rust: `curl https://sh.rustup.rs -sSf | sh`. See <https://rustup.rs> for details.
- Initialize all submodules: `git submodule update --init --recursive`.

### Creating Octrees

In the root of the repo, run `cargo build --release`.
Then use `target/release/build_octree` to generate an octree out of a PLY file.

### SDL client

This is a native client using [SDL2](https://libsdl.org).

1. Install SDL2. For example, on Mac `brew install sdl2`. 
2. Change to the sdl viewer's directory: `cd sdl_viewer/`. 
3. Build with `cargo build --release`. 
4. Run with `../target/release/sdl_viewer <octree directory>`.

In the point cloud viewer, navigate with the keyboard or with the mouse or touchpad. Dragging while pressing the left mouse button rotates, dragging while pressing the right mouse button pans the view. The following keys are bound:

| Key                | Action                        |
| ------------------ | ----------------------------- |
| T                  | Toggle the view to CT mode    |
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

To build and run the `octree_web_viewer` please look into [the `octree_web_viewer` README file](octree_web_viewer/README.md)

## Prior art

This work was inspired by the following projects.

- [Potree](http://potree.org)
- [Megatree](http://wiki.ros.org/megatree)
