#!/bin/bash

set -ex

install_javascript_stuff() {
  curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.35.3/install.sh | bash
  source ~/.nvm/nvm.sh
  nvm install 12.18.0

  sudo apt-key adv --fetch-keys http://dl.yarnpkg.com/debian/pubkey.gpg
  echo "deb http://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
  sudo apt-get update -qq
  sudo apt-get install -y -qq yarn=1.22.4-1

  node -v
  pushd octree_web_viewer/client
  yarn install
  popd

  pushd xray/client
  yarn install
  popd
}

main() {
    install_javascript_stuff

    export PATH="$PATH:$HOME/bin"

    cargo install --force grpcio-compiler
}

main
