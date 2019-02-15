#!/bin/bash

set -ex

main() {
    if [ "$TRAVIS_RUST_VERSION" == "nightly" ]; then
        rustup component add clippy --toolchain=nightly || cargo install --git https://github.com/rust-lang/rust-clippy/ --force clippy
    else
        rustup component add clippy
    fi
}

main
