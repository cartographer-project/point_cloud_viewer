#!/bin/bash

set -ex

main() {
    [[ "$TRAVIS_RUST_VERSION" == "nightly" ]] && nightly_suffix+=("--toolchain=nightly || cargo install --git https://github.com/rust-lang/rust-clippy/ --force clippy")
    rustup component add clippy "${nightly_suffix[@]}"
}

main
