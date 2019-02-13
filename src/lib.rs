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

pub mod color;
pub mod errors;
pub mod generation;
pub mod math;
pub mod octree;
pub mod ply;
pub mod pts;

pub trait InternalIterator {
  fn for_each<F: FnMut(&Point)>(self, _: F);
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
