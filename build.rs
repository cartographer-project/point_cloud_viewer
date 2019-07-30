// Copyright 2016 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use cmake::Config;
use std::env;

fn main() {
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    // Build and install proto3
    Config::new("third_party/protobuf/cmake")
        .define("CMAKE_BUILD_TYPE", "Release")
        .define(
            "CMAKE_INSTALL_PREFIX",
            format!("{}/target/protobuf", manifest_dir),
        )
        .define("CMAKE_POSITION_INDEPENDENT_CODE", "True")
        .define("protobuf_BUILD_TESTS", "False")
        .build();
}
