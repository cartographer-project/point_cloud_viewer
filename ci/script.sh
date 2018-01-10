set -ex

main() {
    cargo build --verbose
    cargo test

    pushd web_viewer
    pushd client
    npm run build
    popd

    cargo build --verbose
    cargo test
    popd

    pushd sdl_viewer
    cargo build --verbose
    cargo test
    popd
}

main
