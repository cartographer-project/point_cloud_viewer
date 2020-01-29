set -ex

main() {
    # octree_web_viewer - client
    pushd octree_web_viewer/client
    npm run build
    popd

    # xray viewer - client
    pushd xray/client
    npm run build
    popd

    # root
    cargo clippy --workspace -- -D warnings
    cargo build --workspace --verbose
    cargo test --workspace
}

main
