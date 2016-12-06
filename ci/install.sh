#!/bin/bash

# `install` phase: install stuff needed for the `script` phase

set -ex

. $(dirname $0)/utils.sh

install_rustup() {
    # uninstall the rust toolchain installed by travis, we are going to use rustup
    sh ~/rust/lib/rustlib/uninstall.sh
    curl https://sh.rustup.rs -sSf | sh -s -- -y --default-toolchain=$TRAVIS_RUST_VERSION
    rustc -V
    cargo -V
}

install_standard_crates() {
    if [ $(host) != "$TARGET" ]; then
        rustup target add $TARGET
    fi
}

configure_cargo() {
    local prefix=$(gcc_prefix)

    if [ ! -z $prefix ]; then
        # information about the cross compiler
        ${prefix}gcc -v

        # tell cargo which linker to use for cross compilation
        mkdir -p .cargo
        cat >>.cargo/config <<EOF
[target.$TARGET]
linker = "${prefix}gcc"
EOF
    fi
}

install_javascript_stuff() {
  pushd client
  npm install
  npm run typings:install
  popd
}

main() {
    install_javascript_stuff
    install_rustup
    install_standard_crates
    configure_cargo
}

main
