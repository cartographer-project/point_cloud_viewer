set -ex

main() {
    # web viewer - client
    pushd web_viewer
    pushd client
    npm run build
    popd

    # root
    cargo build --all --verbose
    cargo test --all
}

main
