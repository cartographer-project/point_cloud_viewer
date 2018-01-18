// Copyright 2016 The Cartographer Authors
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

extern crate cgmath;
extern crate lru_cache;
extern crate point_viewer;
extern crate rand;

/// Unsafe macro to create a static null-terminated c-string for interop with OpenGL.
#[macro_export]
macro_rules! c_str {
    ($s:expr) => {
        concat!($s, "\0").as_ptr() as *const i8
    }
}

mod glhelper;
mod camera;

#[allow(non_upper_case_globals)]
pub mod opengl {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

pub mod box_drawer;
pub mod color;
// TODO(thomasschiwietz): Use 'Color' in the 'Point' struct in src/lib.rs (top level crate)
// instead of using single variables for r,g,b

pub mod graphic;

pub mod node_drawer;

pub use camera::Camera;
