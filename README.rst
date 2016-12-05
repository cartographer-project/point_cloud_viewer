Point viewer
============

This is a standalone project to make viewing massive point clouds easy and
convenient. It was build to serve the needs of the Cartographer_ project, but
is useful in its own right.

.. Cartographer: http://github.com/googlecartographer

Building
========

Client
------

1. Change into the client directory `cd client`.
1. Install npm: `sudo apt-get install npm` or equivalent for your system.
1. Install javascript dependencies: `npm install`.
1. Install typescript module dependencies: `typings install`.
1. Build the client: `tsc`.

Server
------

1. Install Rust: `curl https://sh.rustup.rs -sSf | sh`. See
   [rustup.rs](http://rustup.rs) for details.
1. `cargo build --release`. The binaries will end up in `target/release/`. The
   server binaries includes the client files, so the server is stand alone.

Usage
-----

First, build an octree using `build_octree` from a point source like a PLY or
XYZ file. Then serve it up using `server` - this requires that the client files
have been build and are found.

Prior art
=========

This work was inspired through the following projects. Its focus was on ease
of deployment and speed, which ruled the other projects out.

- Potree_
- Megatree_

.. Potree: http://potree.org
.. Megatree_: http://wiki.ros.org/megatree.
