#!/bin/bash

set -ex

main() {
    rustup component add clippy
}

main
