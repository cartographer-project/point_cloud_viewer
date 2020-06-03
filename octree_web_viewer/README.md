### Web Viewer

The `octree_web_viewer` consists of [TypeScript](https://www.typescriptlang.org) code running in the browser and a web server binary.

To build,

1. Install node.js and yarn. On Mac `brew install yarn`.
2. Change into the web viewer's client directory: `cd client`.
3. Install javascript dependencies: `yarn install`.
4. Build the client: `yarn run build`.


Then build the server: `cargo build --release`.

Serve up the octree using `../target/release/points_web_viewer <octree directory>`, open Chrome to <http://localhost:5433>, navigate with WASD and left-click-drag on the mouse. You can also switch serving different octreees which reside at the same subpath by detailing the octree folder string in the GUI. 
For help and customization arguments, type `../target/release/points_web_viewer --help`. 
The mouse wheel adjusts movement speed.

The client files (HTML and JavaScript) are embedded in the `points_web_viewer` binary, so it is fully stand alone.
