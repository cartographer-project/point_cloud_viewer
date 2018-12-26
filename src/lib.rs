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

// TODO(sirver): Work around suppressing a warning until
// https://github.com/rust-lang-nursery/error-chain/pull/246 is released.
#![allow(renamed_and_removed_lints)]
#![recursion_limit = "1024"]

extern crate byteorder;
extern crate cgmath;
extern crate collision;
#[macro_use]
extern crate error_chain;
extern crate fnv;
extern crate libc;
extern crate num;
extern crate num_traits;
extern crate pbr;
extern crate point_viewer_proto_rust;
extern crate protobuf;
extern crate scoped_pool;
extern crate walkdir;

pub mod color;
pub mod errors;
pub mod math;
pub mod generation;
pub mod octree;
pub mod ply;
pub mod pts;


// Assuming 20 - 40 bytes per point, this translates to 10-20MB per points batch.
const NUM_POINTS_PER_BATCH: i64 = 500_000;

pub trait InternalIterator {
    fn for_each_batch<F: FnMut(&octree::PointData)>(self, F);
    fn size_hint(&self) -> Option<usize>;
}

#[derive(Debug, Clone)]
pub struct Point {
    pub position: cgmath::Vector3<f32>,
    // TODO(sirver): Make color optional, we might not always have it.
    pub color: color::Color<u8>,

    // The intensity of the point if it exists. This value is usually handed through directly by a
    // sensor and has therefore no defined range - or even meaning.
    pub intensity: Option<f32>,
}

pub use point_viewer_proto_rust::proto;
