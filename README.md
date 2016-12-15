# Point viewer

[![build status](https://travis-ci.org/googlecartographer/point_cloud_viewer.svg?branch=master)](https://travis-ci.org/googlecartographer/point_cloud_viewer)

This is a standalone project to make viewing massive point clouds easy and
convenient. It was build to serve the needs of the
[Cartographer](http://github.com/googlecartographer) project, but
is useful in its own right.

## Building


### Client

1. Change into the client directory: `cd client`.
2. Install npm. We strongly suggest using [nvm](https://github.com/creationix/nvm).
3. Install javascript dependencies: `npm install`.
4. Build the client: `npm run build`.

### Server

1. Install Rust: `curl https://sh.rustup.rs -sSf | sh`. See
   <http://rustup.rs> for details.
2. `cargo build --release`. The binaries will end up in `target/release/`. The
   server binaries includes the client files, so the server is stand alone.

If you want to develop and change the `.proto` files you will also need to
[install the code generation from
rust-protobuf](https://github.com/stepancheg/rust-protobuf/). For now, the
generated rust code is checked in and needs to be regenerated manually.

## Usage

First, build an octree using `build_octree` from a point source like a PLY or
XYZ file. Then serve it up using `viewer` - this requires that the client files
have been build and are found.

## Prior art

This work was inspired through the following projects. This project focus is on
ease of deployment and speed.

- [Potree](http://potree.org)
- [Megatree](http://wiki.ros.org/megatree)
