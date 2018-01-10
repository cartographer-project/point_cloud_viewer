#!/bin/bash

set -ex

install_javascript_stuff() {
  curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.31.1/install.sh | bash
  source ~/.nvm/nvm.sh

  nvm install 5.0.0

  pushd web_viewer/client
  npm install
  popd
}

main() {
    install_javascript_stuff

    sudo apt-get install -y \
        libsdl2-2.0 \
        libsdl2-dev \
        libc-dev
}

main
