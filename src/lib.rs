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

#![recursion_limit = "1024"]

extern crate byteorder;
extern crate bytes;
extern crate cgmath;
extern crate num;
extern crate num_traits;
extern crate prost;
#[macro_use]
extern crate prost_derive;
extern crate walkdir;
#[macro_use]
extern crate error_chain;

pub mod math;
pub mod octree;
pub mod ply;
pub mod pts;
pub mod errors;

pub trait InternalIterator {
    fn for_each<F: FnMut(&Point)>(self, F);
    fn size_hint(&self) -> Option<usize>;
}

#[derive(Debug,Clone)]
pub struct Point {
    pub position: math::Vector3f,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/proto.rs"));
}
