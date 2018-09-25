#!/bin/bash

set -ex

install_javascript_stuff() {
  curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.31.1/install.sh | bash
  source ~/.nvm/nvm.sh

  nvm install 5.0.0

  pushd octree_web_viewer/client
  npm install
  popd

  pushd xray/client
  npm install
  popd
}

main() {
    install_javascript_stuff

    bash ci/install_proto3.sh
    export PATH="$PATH:$HOME/bin"

    cargo install --vers 1.4.3 --force protobuf
    cargo install --force grpcio-compiler
}

main
