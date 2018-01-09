#!/bin/bash

set -ex

install_javascript_stuff() {
  curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.31.1/install.sh | bash
  source ~/.nvm/nvm.sh

  nvm install 5.0.0

  pushd web_viewer/client
  npm install
  popd

  sudo apt-get install libsdl2-2.0
  sudo apt-get install libsdl2-dev
}

# install_point_cloud_viewer_deps() {
  # sudo apt-get install libsdl2-2.0
  # sudo apt-get install libsdl2-dev
# }

main() {
    install_javascript_stuff
    # install_point_cloud_viewer_deps
}

main
