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
extern crate cgmath;
extern crate pbr;
extern crate walkdir;
extern crate protobuf;
#[macro_use]
extern crate error_chain;
#[macro_use]
extern crate nom;

pub mod math;
pub mod octree;
pub mod ply;
pub mod pts;
pub mod point_stream;
pub mod errors;

#[derive(Debug)]
pub struct Point {
    pub position: math::Vector3f,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

pub mod proto;
