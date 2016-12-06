set -ex

. $(dirname $0)/utils.sh

main() {
    pushd client
    npm run build
    popd

    cargo build --target $TARGET --verbose
    cargo test --target $TARGET
}

main
