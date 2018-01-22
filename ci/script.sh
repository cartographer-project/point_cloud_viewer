set -ex

main() {
    # web viewer - client
    pushd web_viewer/client
    npm run build
    popd

    # root
    cargo build --all --verbose
    cargo test --all
}

main
