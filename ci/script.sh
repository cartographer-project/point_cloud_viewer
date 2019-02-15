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
    cargo clippy --all -- -D warnings
    cargo build --all --verbose
    cargo test --all
}

main
