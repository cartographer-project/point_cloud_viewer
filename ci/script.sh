set -ex

main() {
    source ~/.nvm/nvm.sh
    nvm use $(cat .nvmrc)

    # octree_web_viewer - client
    pushd octree_web_viewer/client
    yarn run build
    popd

    # xray viewer - client
    pushd xray/client
    yarn run build
    popd

    # root
    cargo clippy --workspace -- -D warnings
    cargo build --workspace --verbose --all-targets
    cargo test --workspace
}

main
