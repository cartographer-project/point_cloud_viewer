set -ex

main() {
    pushd client
    npm run build
    popd

    cargo build --verbose
    cargo test
}

main
