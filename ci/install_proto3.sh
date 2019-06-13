#!/bin/sh

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -o errexit
set -o verbose

VERSION="v3.6.1"
ROOT_DIR="$( cd "$( dirname "$( dirname "$0" )" )" >/dev/null 2>&1 && pwd )"
PROTOBUF_DIR=${ROOT_DIR}/target/protobuf
TMPDIR="$( mktemp -d )"

# Build and install proto3.
git clone https://github.com/google/protobuf.git ${TMPDIR}
git clone https://github.com/google/protobuf.git
cd ${TMPDIR}
git checkout tags/${VERSION}
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_INSTALL_PREFIX=${PROTOBUF_DIR} \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -Dprotobuf_BUILD_TESTS=OFF \
  ../cmake
ninja
ninja install
rm -rf ${TMPDIR}
