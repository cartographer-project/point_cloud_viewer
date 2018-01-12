set -ex

main() {
    # root
    cargo build --verbose
    cargo test

    # grpc
    pushd grpc
    cargo build --verbose
    cargo test

    # web viewer - client
    pushd web_viewer
    pushd client
    npm run build
    popd

    # web viewer - server
    cargo build --verbose
    cargo test
    popd

    # sdl viewer
    pushd sdl_viewer
    cargo build --verbose
    cargo test
    popd
}

main
