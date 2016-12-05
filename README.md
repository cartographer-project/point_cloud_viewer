# Point viewer

This is a standalone project to make viewing massive point clouds easy and
convenient. It was build to serve the needs of the
[Cartographer](http://github.com/googlecartographer) project, but
is useful in its own right.

## Building


### Client

1. Change into the client directory: `cd client`.
2. Install npm: `sudo apt-get install npm` or equivalent for your system.
3. Install javascript dependencies: `npm install`.
4. Install typescript module dependencies: `typings install`.
5. Build the client: `tsc`.

### Server

1. Install Rust: `curl https://sh.rustup.rs -sSf | sh`. See
   <http://rustup.rs> for details.
2. `cargo build --release`. The binaries will end up in `target/release/`. The
   server binaries includes the client files, so the server is stand alone.

## Usage

First, build an octree using `build_octree` from a point source like a PLY or
XYZ file. Then serve it up using `server` - this requires that the client files
have been build and are found.

## Prior art

This work was inspired through the following projects. This project focus is on
ease of deployment and speed.

- [Potree](http://potree.org)
- [Megatree](http://wiki.ros.org/megatree)
