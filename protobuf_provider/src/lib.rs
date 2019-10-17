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

use itertools::Itertools;
use std::env;
use std::ffi::OsStr;
use std::path::Path;
use std::process::Command;
use std::str;

pub const PROTOBUF_BIN_PATH: &str = concat!(env!("OUT_DIR"), "/bin");
pub const PROTOBUF_LIB_PATH: &str = concat!(env!("OUT_DIR"), "/lib");
pub const PROTOBUF_INCLUDE_PATH: &str = concat!(env!("OUT_DIR"), "/include");

pub struct ScopedProtocPath {
    old_path: String,
}

impl Default for ScopedProtocPath {
    fn default() -> Self {
        let old_path = env::var("PATH").unwrap_or_else(|_| "".to_string());
        env::set_var("PATH", PROTOBUF_BIN_PATH);
        Self { old_path }
    }
}

impl Drop for ScopedProtocPath {
    fn drop(&mut self) {
        env::set_var("PATH", &self.old_path);
    }
}

pub enum ProtocOutputType {
    Cc,
    Py,
}

pub fn compile_proto(
    import_paths: &[impl AsRef<Path>],
    file_path: impl AsRef<Path>,
    output_path: impl AsRef<Path>,
    output_types: &[ProtocOutputType],
) {
    let _protoc_path = ScopedProtocPath::default();

    let mut args = vec![file_path.as_ref().as_os_str()];
    args.extend(
        vec![OsStr::new("--proto_path"); import_paths.len()]
            .into_iter()
            .interleave(import_paths.iter().map(|p| p.as_ref().as_os_str())),
    );
    args.extend(
        output_types
            .iter()
            .map(|o| {
                OsStr::new(match o {
                    ProtocOutputType::Cc => "--cpp_out",
                    ProtocOutputType::Py => "--python_out",
                })
            })
            .interleave(vec![output_path.as_ref().as_os_str(); output_types.len()].into_iter()),
    );
    let output = Command::new("protoc")
        .args(args)
        .output()
        .expect("Error executing protoc – does it exist in the expected location?");

    let error_msg = str::from_utf8(&output.stderr)
        .unwrap_or("[Not valid UTF-8]")
        .trim_end();
    assert!(output.status.success(), "protoc complains: {}", error_msg);
}
