### Web Viewer

The `octree_web_viewer` consists of [TypeScript](https://www.typescriptlang.org) code running in the browser and a web server binary.

To build,

1. Change into the web viewer's client directory: `cd octree_web_viewer/client`.
2. Install npm. We strongly suggest using [nvm](https://github.com/creationix/nvm). On Mac `brew install nvm`. 
3. Install node version 8: `nvm install 8`. Change to the web viewer's client directory: `cd client`, then set node version to 8: `nvm use 8`. 
4. Install javascript dependencies: `yarn install`.
5. Build the client: `yarn build`.


Then build the server: `cargo build --release`.

Serve up the octree using `../target/release/points_web_viewer <octree directory>`, open Chrome to <http://localhost:5433>, navigate with WASD and left-click-drag on the mouse. You can also switch serving different octreees which reside at the same subpath by detailing the octree folder string in the GUI. 
For help and customization arguments, type `../target/release/points_web_viewer --help`. 
The mouse wheel adjusts movement speed.

The client files (HTML and JavaScript) are embedded in the `points_web_viewer` binary, so it is fully stand alone.
