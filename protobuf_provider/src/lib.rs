// Copyright 2018 Google Inc.
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

pub const PROTOBUF_BIN_PATH: &str = concat!(env!("OUT_DIR"), "/bin");
pub const PROTOBUF_LIB_PATH: &str = concat!(env!("OUT_DIR"), "/lib");
pub const PROTOBUF_INCLUDE_PATH: &str = concat!(env!("OUT_DIR"), "/include");

pub struct ScopedProtocPath {
    old_path: String,
}

impl Default for ScopedProtocPath {
    fn default() -> Self {
        let old_path = std::env::var("PATH").unwrap_or_else(|_| "".to_string());
        std::env::set_var("PATH", PROTOBUF_BIN_PATH);
        Self { old_path }
    }
}

impl Drop for ScopedProtocPath {
    fn drop(&mut self) {
        std::env::set_var("PATH", &self.old_path);
    }
}
